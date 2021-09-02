/*
    PIC16f84 Focuser

    Copyright (C) 2021 Patrick Molenaar

    Based on DeepSkyDad driver.
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

#pragma once

#include "indifocuser.h"

#include <chrono>

class PIC16f84Focuser : public INDI::Focuser
{
    public:
        PIC16f84Focuser();
        virtual ~PIC16f84Focuser() override = default;

        typedef enum { S1, S2 } FocusStepMode;
        typedef enum { VERY_SLOW, SLOW, MEDIUM, FAST, VERY_FAST } FocusSpeedMode;

        const char * getDefaultName() override;

        /**
         * \brief Initilize focuser properties. It is recommended to call this function within
         * initProperties() of your primary device
         * \param groupName Group or tab name to be used to define focuser properties.
         */
        virtual bool initProperties() override;

        /**
         * @brief updateProperties Define or Delete Rotator properties based on the connection status of the base device
         * @return True if successful, false otherwise.
         */
        virtual bool updateProperties() override;
        virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;
        virtual bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n) override;

        static void timedMoveHelper(void * context);

        void debugTriggered(bool enable) override;

    protected:
        /**
         * @brief Handshake Try to communicate with Focuser and see if there is a valid response.
         * @return True if communication is successful, false otherwise.
         */
        virtual bool Handshake() override;

        /**
         * @brief MoveFocuser Move focuser in a specific direction and speed for period of time.
         * @param dir Direction of motion. Inward or Outward
         * @param speed Speed of motion
         * @param duration Timeout in milliseconds.
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;

        /**
         * @brief MoveAbsFocuser Move to an absolute target position
         * @param targetTicks target position
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;

        /**
         * @brief MoveRelFocuser Move focuser for a relative amount of ticks in a specific direction
         * @param dir Directoin of motion
         * @param ticks steps to move
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;

        /**
         * @brief SyncFocuser Set the supplied position as the current focuser position
         * @param ticks target position
         * @return IPS_OK if focuser position is now set to ticks. IPS_ALERT for problems.
         */
        virtual bool SyncFocuser(uint32_t ticks) override;

        /**
         * @brief ReverseFocuser Reverse focuser motion direction
         * @param enabled If true, normal default focuser motion is reversed. If false, the direction is set to the default focuser motion.
         * @return True if successful, false otherwise.
         */
        virtual bool ReverseFocuser(bool enabled) override;

        /**
         * @brief AbortFocuser Abort Focuser motion
         * @return True if successful, false otherwise.
         */
        virtual bool AbortFocuser() override;

        /**
         * @brief TimerHit Primary Loop called every POLLMS milliseconds (set in Options) to check
         * on the focuser status like read position, temperature.. and check if the focuser reached the required position.
         */
        virtual void TimerHit() override;

        /**
         * @brief saveConfigItems save focuser properties defined in the interface in config file
         * @param fp pointer to config file
         * @return Always return true
         */
        virtual bool saveConfigItems(FILE * fp) override;

        /**
         * @brief SetFocuserMaxPosition Set Focuser Maximum position limit in the hardware.
         * @param ticks maximum steps permitted
         * @return True if successful, false otherwise.
         * @note If setting maximum position limit in the hardware is not available or not supported, do not override this function as the default
         * implementation will always return true.
         */
        virtual bool SetFocuserMaxPosition(uint32_t ticks) override;

    private:
        bool Ack();
        /**
         * @brief sendCommand Send a string command to PIC16f84 focuser.
         * @param cmd Command to be sent in format "[CMD]"
         * @param res If not nullptr, the function will read until it detects the response in format "(RES)"
         *        if nullptr, no read back is done and the function returns true.
         * @return True if successful, false otherwise.
         */
        bool sendCommand(const char * cmd, char * res = nullptr);
        bool sendCommandSet(const char * cmd);

        // Get initial focuser parameter when we first connect
        void GetFocusParams();
        bool readStepMode();
        bool readSpeedMode();
        bool readPosition();
        bool readMaxPosition();
        bool isMoving();
        void timedMoveCallback();
        bool MoveFocuser(uint32_t position);
        int msleep(long milliseconds);

        double targetPos { 0 }, lastPos { 0 };
        bool moveAborted = false;

        // Step mode
        ISwitch StepModeS[9];
        ISwitchVectorProperty StepModeSP;

        // Speed mode
        ISwitch SpeedModeS[5];
        ISwitchVectorProperty SpeedModeSP;

        // Response Buffer
        static const uint8_t DSD_RES { 15 };
        static const char DSD_DEL { ')' };
        static const uint8_t DSD_TIMEOUT { 3 };
};
