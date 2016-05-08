using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CanTest
{
    class Data_MCP2515_Receiver
    {
        // DATA FOR RECEIVER
        private cONTROL_REGISTER_CANINTE_VALUE control_register_caninte_value;
        private cONTROL_REGISTER_RXB0CTRL_VALUE control_register_rxb0cntrl_value;

        public struct cONTROL_REGISTER_CANINTE_VALUE
        {
            public byte INTE;
        }

        public struct cONTROL_REGISTER_RXB0CTRL_VALUE
        {
            public byte RXB0CTRL;
        }

        public Data_MCP2515_Receiver()
        {
            // Set values for interrupts on int pin 
            control_register_caninte_value.INTE = 0x01; // Enable interrupt when rx buffer 0 full

            // Set values for mask and filter on buffer 0
            control_register_rxb0cntrl_value.RXB0CTRL = 0x60; // Disable mask and filter on buffer 0 rx -> Receive everything
        }


        public cONTROL_REGISTER_CANINTE_VALUE CONTROL_REGISTER_CANINTE_VALUE
        {
            get
            {
                return control_register_caninte_value;
            }

            set
            {
                control_register_caninte_value = value;
            }
        }

        public cONTROL_REGISTER_RXB0CTRL_VALUE CONTROL_REGISTER_RXB0CTRL_VALUE
        {
            get
            {
                return control_register_rxb0cntrl_value;
            }

            set
            {
                control_register_rxb0cntrl_value = value;
            }
        }
    }
}
