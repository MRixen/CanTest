using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CanTest
{
    class Data_MCP2515_Sender
    {
        // DATA FOR SENDER
        private cONTROL_REGISTER_CANINTE_VALUE_SENDER control_register_caninte_value_sender;
        private cONTROL_REGISTER_TXB0CTRL_VALUE control_register_txb0ctrl_value;


        public Data_MCP2515_Sender()
        {
            // Set values for interrupts on int pin 
            control_register_caninte_value_sender.INTE = 0x00; // Disable interrupt 

            // Set values for mask and filter on buffer 0
            control_register_txb0ctrl_value.TXB0CTRL = 0x00; // Disable mask and filter on buffer 0 tx -> Lowest priority for buffer 0
        }

        public struct cONTROL_REGISTER_CANINTE_VALUE_SENDER
        {
            public byte INTE;
        }

        public struct cONTROL_REGISTER_TXB0CTRL_VALUE
        {
            public byte TXB0CTRL;
        }

        public cONTROL_REGISTER_TXB0CTRL_VALUE CONTROL_REGISTER_TXB0CTRL_VALUE
        {
            get
            {
                return control_register_txb0ctrl_value;
            }

            set
            {
                control_register_txb0ctrl_value = value;
            }
        }


        public cONTROL_REGISTER_CANINTE_VALUE_SENDER CONTROL_REGISTER_CANINTE_VALUE_SENDER
        {
            get
            {
                return control_register_caninte_value_sender;
            }

            set
            {
                control_register_caninte_value_sender = value;
            }
        }
    }
}
