/*
    PIC16f84 Focuser

    Copyright (C) 2021 Patrick Molenaar

    Based on Moonlite driver.
    Copyright (C) 2013-2019 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "pic16f84focuser.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

static std::unique_ptr<PIC16f84Focuser> pic16f84Focuser(new PIC16f84Focuser());

PIC16f84Focuser::PIC16f84Focuser()
{
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_SYNC | FOCUSER_CAN_REVERSE | FOCUSER_CAN_ABORT);
}

void PIC16f84Focuser::debugTriggered(bool enable)
{
    INDI_UNUSED(enable);
    // tty_set_debug(enable ? 1 : 0);
}

bool PIC16f84Focuser::initProperties()
{
    INDI::Focuser::initProperties();

    // Step Mode
    IUFillSwitch(&StepModeS[S2], "S2", "Half Step", ISS_OFF); // High power, high precision stepping
    IUFillSwitch(&StepModeS[S1], "S1", "Full Step", ISS_OFF); // High power, low precision stepping
    IUFillSwitchVector(&StepModeSP, StepModeS, 2, getDeviceName(), "Step Mode", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Speed Mode
    IUFillSwitch(&SpeedModeS[VERY_SLOW], "VERY_SLOW", "Very slow", ISS_OFF);
    IUFillSwitch(&SpeedModeS[SLOW], "SLOW", "Slow", ISS_OFF);
    IUFillSwitch(&SpeedModeS[MEDIUM], "MEDIUM", "Medium", ISS_OFF);
    IUFillSwitch(&SpeedModeS[FAST], "FAST", "Fast", ISS_OFF);
    IUFillSwitch(&SpeedModeS[VERY_FAST], "VERY_FAST", "Very fast", ISS_OFF);
    IUFillSwitchVector(&SpeedModeSP, SpeedModeS, 5, getDeviceName(), "Speed Mode", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min = 0.;
    FocusRelPosN[0].max = 65535.;
    FocusRelPosN[0].value = 0.;
    FocusRelPosN[0].step = 13.;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 65535.;
    FocusAbsPosN[0].value = 32767.;
    FocusAbsPosN[0].step = 327.;

    FocusMaxPosN[0].min = 0.;
    FocusMaxPosN[0].max = 65535.;
    FocusMaxPosN[0].value = 65535.;
    FocusMaxPosN[0].step = 327.;

    FocusSyncN[0].min = 0.;
    FocusSyncN[0].max = 65535.;
    FocusSyncN[0].value = 32767.;
    FocusSyncN[0].step = 327.;

    setDefaultPollingPeriod(2000);
    addDebugControl();

    return true;
}

bool PIC16f84Focuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(&StepModeSP);
        defineProperty(&SpeedModeSP);

        GetFocusParams();

        LOG_INFO("PIC16f84Focuser parameters updated, focuser ready for use.");
    }
    else
    {
        deleteProperty(StepModeSP.name);
        deleteProperty(SpeedModeSP.name);
    }

    return true;
}

bool PIC16f84Focuser::Handshake()
{
    if (Ack())
    {
        LOG_INFO("PIC16f84Focuser is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO(
        "Error retrieving data from pic16f84Focuser, please ensure pic16f84Focuser controller is powered and the port is correct.");
    return false;
}

const char * PIC16f84Focuser::getDefaultName()
{
    return "PIC16f84 Focuser";
}

bool PIC16f84Focuser::Ack()
{
    sleep(2);

    char res[DSD_RES] = {0};
    if (!sendCommand("[GPOS]", res) && !sendCommand("[GPOS]", res)) // try twice
    {
        LOG_ERROR("ACK - getPosition failed");
        return false;
    }

    int32_t pos;
    int rc = sscanf(res, "(%d)", &pos);

    if (rc <= 0)
    {
        LOG_ERROR("ACK - getPosition failed");
        return false;
    }

    return true;
}

bool PIC16f84Focuser::readStepMode()
{
    char res[DSD_RES] = {0};

    if (sendCommand("[GSTP]", res) == false)
        return false;

    if (strcmp(res, "(1)") == 0)
        StepModeS[S1].s = ISS_ON;
    else if (strcmp(res, "(2)") == 0)
        StepModeS[S2].s = ISS_ON;
    else
    {
        LOGF_ERROR("Unknown error: focuser step value (%s)", res);
        return false;
    }

    StepModeSP.s = IPS_OK;
    return true;
}

bool PIC16f84Focuser::readSpeedMode()
{
    char res[DSD_RES] = {0};

    if (sendCommand("[GSPD]", res) == false)
        return false;

    if (strcmp(res, "(1)") == 0)
        SpeedModeS[VERY_SLOW].s = ISS_ON;
    else if (strcmp(res, "(2)") == 0)
        SpeedModeS[SLOW].s = ISS_ON;
    else if (strcmp(res, "(3)") == 0)
        SpeedModeS[MEDIUM].s = ISS_ON;
    else if (strcmp(res, "(4)") == 0)
        SpeedModeS[FAST].s = ISS_ON;
    else if (strcmp(res, "(5)") == 0)
        SpeedModeS[VERY_FAST].s = ISS_ON;
    else
    {
        LOGF_ERROR("Unknown error: focuser speed value (%s)", res);
        return false;
    }

    SpeedModeSP.s = IPS_OK;
    return true;
}

bool PIC16f84Focuser::readPosition()
{
    char res[DSD_RES] = {0};

    if (sendCommand("[GPOS]", res) == false)
        return false;

    int32_t pos;
    int rc = sscanf(res, "(%d)", &pos);

    if (rc > 0)
        FocusAbsPosN[0].value = pos;
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool PIC16f84Focuser::readMaxPosition()
{
    char res[DSD_RES] = {0};

    if (sendCommand("[GMXP]", res) == false)
        return false;

    uint32_t steps = 0;
    int rc = sscanf(res, "(%d)", &steps);
    if (rc > 0)
    {
        FocusMaxPosN[0].value = steps;
        FocusMaxPosNP.s = IPS_OK;
    }
    else
    {
        LOGF_ERROR("Unknown error: maximum position value (%s)", res);
        return false;
    }

    return true;
}

bool PIC16f84Focuser::isMoving()
{
    char res[DSD_RES] = {0};

    if (sendCommand("[GMOV]", res) == false)
        return false;

    if (strcmp(res, "(1)") == 0)
        return true;
    else if (strcmp(res, "(0)") == 0)
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}

bool PIC16f84Focuser::SyncFocuser(uint32_t ticks)
{
    char cmd[DSD_RES] = {0};
    snprintf(cmd, DSD_RES, "[SPOS%05d]", ticks);
    char res[DSD_RES] = {0};
    return sendCommand(cmd, res);
}

bool PIC16f84Focuser::ReverseFocuser(bool enabled)
{
    char cmd[DSD_RES] = {0};
    snprintf(cmd, DSD_RES, "[SREV%d]", enabled ? 1 : 0);
    char res[DSD_RES] = {0};
    return sendCommand(cmd, res);
}

bool PIC16f84Focuser::MoveFocuser(uint32_t position)
{
    char cmd[DSD_RES] = {0};
    char res[DSD_RES] = {0};
    snprintf(cmd, DSD_RES, "[STRG%06d]", position);
    // Set Position First
    if (sendCommand(cmd, res) == false)
        return false;

    if(strcmp(res, "!101)") == 0)
    {
        LOG_ERROR("MoveFocuserFailed - requested movement too big. You can increase the limit by changing the value of Max. movement.");
        return false;
    }

    // Now start motion toward position
    if (sendCommand("[SMOV]", res) == false)
        return false;

    return true;
}

bool PIC16f84Focuser::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Focus Step Mode
        if (strcmp(StepModeSP.name, name) == 0)
        {
            int current_mode = IUFindOnSwitchIndex(&StepModeSP);

            IUUpdateSwitch(&StepModeSP, states, names, n);

            int target_mode = IUFindOnSwitchIndex(&StepModeSP);

            if (current_mode == target_mode)
            {
                StepModeSP.s = IPS_OK;
                IDSetSwitch(&StepModeSP, nullptr);
                return true;
            }

            char cmd[DSD_RES] = {0};

            if(target_mode == 0)
                target_mode = 1;
            else if(target_mode == 1)
                target_mode = 2;
            else if(target_mode == 2)
                target_mode = 4;

            snprintf(cmd, DSD_RES, "[SSTP%d]", target_mode);
            bool rc = sendCommandSet(cmd);
            if (!rc)
            {
                IUResetSwitch(&StepModeSP);
                StepModeS[current_mode].s = ISS_ON;
                StepModeSP.s              = IPS_ALERT;
                IDSetSwitch(&StepModeSP, nullptr);
                return false;
            }

            StepModeSP.s = IPS_OK;
            IDSetSwitch(&StepModeSP, nullptr);
            return true;
        }

        // Focus Speed Mode
        if (strcmp(SpeedModeSP.name, name) == 0)
        {
            int current_mode = IUFindOnSwitchIndex(&SpeedModeSP);

            IUUpdateSwitch(&SpeedModeSP, states, names, n);

            int target_mode = IUFindOnSwitchIndex(&SpeedModeSP);

            if (current_mode == target_mode)
            {
                SpeedModeSP.s = IPS_OK;
                IDSetSwitch(&SpeedModeSP, nullptr);
                return true;
            }

            char cmd[DSD_RES] = {0};

            if(target_mode == 0)
                target_mode = 1;
            else if(target_mode == 1)
                target_mode = 2;
            else if(target_mode == 2)
                target_mode = 3;
            else if(target_mode == 3)
                target_mode = 4;
            else if(target_mode == 4)
                target_mode = 5;

            snprintf(cmd, DSD_RES, "[SSPD%d]", target_mode);
            bool rc = sendCommandSet(cmd);
            if (!rc)
            {
                IUResetSwitch(&SpeedModeSP);
                SpeedModeS[current_mode].s = ISS_ON;
                SpeedModeSP.s              = IPS_ALERT;
                IDSetSwitch(&SpeedModeSP, nullptr);
                return false;
            }

            SpeedModeSP.s = IPS_OK;
            IDSetSwitch(&SpeedModeSP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool PIC16f84Focuser::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Max. position
        if (strcmp(name, FocusMaxPosNP.name) == 0)
        {
            IUUpdateNumber(&FocusMaxPosNP, values, names, n);
            char cmd[DSD_RES] = {0};
            snprintf(cmd, DSD_RES, "[SMXP%05d]", static_cast<int>(FocusMaxPosN[0].value));
            bool rc = sendCommandSet(cmd);
            if (!rc)
            {
                FocusMaxPosNP.s = IPS_ALERT;
                return false;
            }

            FocusMaxPosNP.s = IPS_OK;
            IDSetNumber(&FocusMaxPosNP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void PIC16f84Focuser::GetFocusParams()
{
    IUResetSwitch(&StepModeSP);
    IUResetSwitch(&SpeedModeSP);

    if (readPosition())
        IDSetNumber(&FocusAbsPosNP, nullptr);

    if (readStepMode())
        IDSetSwitch(&StepModeSP, nullptr);

    if (readSpeedMode())
        IDSetSwitch(&SpeedModeSP, nullptr);

    if (readMaxPosition())
        IDSetNumber(&FocusMaxPosNP, nullptr);
}

IPState PIC16f84Focuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    // either go all the way in or all the way out
    // then use timer to stop
    if (dir == FOCUS_INWARD)
        MoveFocuser(0);
    else
        MoveFocuser(FocusMaxPosN[0].value);

    IEAddTimer(duration, &PIC16f84Focuser::timedMoveHelper, this);
    return IPS_BUSY;
}

void PIC16f84Focuser::timedMoveHelper(void * context)
{
    static_cast<PIC16f84Focuser*>(context)->timedMoveCallback();
}

void PIC16f84Focuser::timedMoveCallback()
{
    AbortFocuser();
    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    FocusTimerNP.s = IPS_IDLE;
    FocusTimerN[0].value = 0;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusTimerNP, nullptr);
}


IPState PIC16f84Focuser::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    if (!MoveFocuser(targetPos))
        return IPS_ALERT;

    return IPS_BUSY;
}

IPState PIC16f84Focuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t newPosition = 0;

    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosN[0].value - ticks;
    else
        newPosition = FocusAbsPosN[0].value + ticks;

    // Clamp
    newPosition = std::max(0, std::min(static_cast<int32_t>(FocusAbsPosN[0].max), newPosition));
    if (!MoveAbsFocuser(newPosition))
        return IPS_ALERT;

    return IPS_BUSY;
}

void PIC16f84Focuser::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    bool rc = readPosition();
    if (rc)
    {
        if (fabs(lastPos - FocusAbsPosN[0].value) > 5)
        {
            IDSetNumber(&FocusAbsPosNP, nullptr);
            lastPos = FocusAbsPosN[0].value;
        }
    }

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        msleep(100); // Sleep for a short moment to give serial channel time to settle

        if (!isMoving())
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastPos = FocusAbsPosN[0].value;

            if(moveAborted)
            {
                LOG_INFO("Move aborted.");
            }
            else
            {
                LOG_INFO("Focuser reached requested position.");
            }

            moveAborted = false;
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

bool PIC16f84Focuser::AbortFocuser()
{
    moveAborted = true;
    return sendCommand("[STOP]");
}

bool PIC16f84Focuser::saveConfigItems(FILE * fp)
{
    Focuser::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &StepModeSP);
    IUSaveConfigSwitch(fp, &SpeedModeSP);

    return true;
}

bool PIC16f84Focuser::sendCommand(const char * cmd, char * res)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if ((rc = tty_nread_section(PortFD, res, DSD_RES, DSD_DEL, DSD_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return true;
}

bool PIC16f84Focuser::sendCommandSet(const char * cmd)
{
    char res[DSD_RES] = {0};

    if (sendCommand(cmd, res) == false)
        return false;

    return strcmp(res, "(OK)") == 0;
}

bool PIC16f84Focuser::SetFocuserMaxPosition(uint32_t ticks)
{
    char cmd[DSD_RES] = {0};

    snprintf(cmd, DSD_RES, "[SMXP%d]", ticks);

    if (sendCommandSet(cmd))
    {
        SyncPresets(ticks);
        return true;
    }

    return false;
}

// sleep for a number of milliseconds
int PIC16f84Focuser::msleep(long duration)
{
    struct timespec ts;
    int res;

    if (duration < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = duration / 1000;
    ts.tv_nsec = (duration % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}
