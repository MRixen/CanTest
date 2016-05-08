using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CanTest
{
    class MCP2515
    {
        // CONTROL REGISTER
        private byte cONTROL_REGISTER_BFPCTRL = 0x0C;
        private byte cONTROL_REGISTER_TXRTSCTRL = 0x0D;
        private byte cONTROL_REGISTER_CANSTAT = 0x0E;
        private byte cONTROL_REGISTER_CANCTRL = 0x0F;
        private byte cONTROL_REGISTER_TEC = 0x1C;
        private byte cONTROL_REGISTER_REGISTER_REC = 0x1D;
        private byte cONTROL_REGISTER_CNF3 = 0x28;
        private byte cONTROL_REGISTER_CNF2 = 0x29;
        private byte cONTROL_REGISTER_CNF1 = 0x2A;
        private byte cONTROL_REGISTER_CANINTE = 0x2B;
        private byte cONTROL_REGISTER_CANINTF = 0x2C;
        private byte cONTROL_REGISTER_EFLG = 0x2D;
        private byte cONTROL_REGISTER_TXB0CTRL = 0x30;
        private byte cONTROL_REGISTER_TXB1CTRL = 0x40;
        private byte cONTROL_REGISTER_TXB2CTRL = 0x50;
        private byte cONTROL_REGISTER_RXB0CTRL = 0x60;
        private byte cONTROL_REGISTER_RXB1CTRL = 0x70;

        // REGISTER
        private byte rEGISTER_TXB0SIDH = 0x31;
        private byte rEGISTER_TXB0SIDL = 0x32;
        private byte rEGISTER_TXB0DLC = 0x35;
        private byte rEGISTER_TXB0D0 = 0x36;
        private byte rEGISTER_TXB0D1 = 0x37;
        private byte rEGISTER_TXB0D2 = 0x38;
        private byte rEGISTER_TXB0D3 = 0x39;
        private byte rEGISTER_TXB0D4 = 0x3A;
        private byte rEGISTER_TXB0D5 = 0x3B;
        private byte rEGISTER_TXB0D6 = 0x3C;
        private byte rEGISTER_TXB0D7 = 0x3D;

        private byte rEGISTER_RXB0D0 = 0x66;
        private byte rEGISTER_RXB0D1 = 0x67;
        private byte rEGISTER_RXB0D2 = 0x68;
        private byte rEGISTER_RXB0D3 = 0x69;
        private byte rEGISTER_RXB0D4 = 0x6A;
        private byte rEGISTER_RXB0D5 = 0x6B;
        private byte rEGISTER_RXB0D6 = 0x6C;
        private byte rEGISTER_RXB0D7 = 0x6D;

        // SPI INSTRUCTIONS
        private byte sPI_INSTRUCTION_RESET = 0xC0;
        private byte sPI_INSTRUCTION_READ = 0x03;
        private byte sPI_INSTRUCTION_READ_RX_BUFFER0_SIDH = 0x90;
        private byte sPI_INSTRUCTION_READ_RX_BUFFER0_D0 = 0x92;
        private byte sPI_INSTRUCTION_WRITE = 0x02;
        private byte sPI_INSTRUCTION_LOAD_TX_BUFFER0_ID = 0x40;
        private byte sPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA = 0x41;
        private byte sPI_INSTRUCTION_RTS_BUFFER0 = 0x81;
        private byte sPI_INSTRUCTION_RTS_BUFFER1 = 0x82;
        private byte sPI_INSTRUCTION_RTS_BUFFER2 = 0x84;
        private byte sPI_INSTRUCTION_READ_STATUS = 0xC0;
        private byte sPI_INSTRUCTION_RX_STATUS = 0xC0;
        private byte sPI_INSTRUCTION_BIT_MODIFY = 0xC0;

        // GLOBAL DATA
        private cONTROL_REGISTER_CNFx_VALUE control_register_cnfx_value;
        private cONTROL_REGISTER_CANSTAT_VALUE control_register_canstat_value;
        private cONTROL_REGISTER_CANCTRL_VALUE control_register_canctrl_value;
        private cONTROL_REGISTER_CANINTF_VALUE control_register_canintf_value;
        private rEGISTER_TXB0SIDL_VALUE register_txb0sidl_value;
        private rEGISTER_TXB0SIDH_VALUE register_txb0sidh_value;
        private byte[] rEGISTER_TXB0Dx = new byte[8];
        private byte[] rEGISTER_RXB0Dx = new byte[8];
        private byte messageSizeAdxl;

        public struct cONTROL_REGISTER_CANSTAT_VALUE
        {
            public byte NORMAL_MODE;
            public byte SLEEP_MODE;
            public byte LOOPBACK_MODE;
            public byte LISTEN_ONLY_MODE;
            public byte CONFIGURATION_MODE;
        }

        public struct rEGISTER_TXB0SIDL_VALUE
        {
            public byte identifier_X;
            public byte identifier_Y;
            public byte identifier_Z;
        }

        public struct rEGISTER_TXB0SIDH_VALUE
        {
            public byte identifier_X;
            public byte identifier_Y;
            public byte identifier_Z;
        }

        public struct cONTROL_REGISTER_CANCTRL_VALUE
        {
            public byte NORMAL_MODE;
            public byte SLEEP_MODE;
            public byte LOOPBACK_MODE;
            public byte LISTEN_ONLY_MODE;
            public byte CONFIGURATION_MODE;
        }

        public struct cONTROL_REGISTER_CANINTF_VALUE
        {
            public byte RESET_RX0IF;
            public byte RESET_RX1IF;
            public byte RESET_TX0IF;
            public byte RESET_TX1IF;
            public byte RESET_TX2IF;
            public byte RESET_ERRIF;
            public byte RESET_WAKIF;
            public byte RESET_MERRIF;
            public byte RESET_ALL_IF;
        }

        public struct cONTROL_REGISTER_CNFx_VALUE
        {
            public byte CNF1;
            public byte CNF2;
            public byte CNF3;
        }



        public struct cONTROL_REGISTER_TXB0CTRL_VALUE
        {
            public byte TXB0CTRL;
        }



        public MCP2515()
        {
            configureGlobalData();
            configureUserData();
        }

        private void configureUserData()
        {
            // Set values for bit timing
            // Fosc = 8Mhz
            // Tosc = 125ns
            control_register_cnfx_value.CNF1 = 0x03; // Baud rate prescaler calculated with application (Fosc = 8Mhz and CANspeed = 125kHz)
            control_register_cnfx_value.CNF2 = 0xB8; // BTLMODE = 1 (PHaseSegment 2 is configured with CNF 3) and PhaseSegment 1 = 8xTQ (7+1)
            control_register_cnfx_value.CNF3 = 0x05; // Set PhaseSegment 2 = 6xTQ (5+1)
        }

        private void configureGlobalData()
        {
            // Set values for mode state
            control_register_canstat_value.NORMAL_MODE = 0x00;
            control_register_canstat_value.SLEEP_MODE = 0x20;
            control_register_canstat_value.LOOPBACK_MODE = 0x40;
            control_register_canstat_value.LISTEN_ONLY_MODE = 0x60;
            control_register_canstat_value.CONFIGURATION_MODE = 0x80;

            // Set values for mode control
            control_register_canctrl_value.NORMAL_MODE = 0x00;
            control_register_canctrl_value.SLEEP_MODE = 0x20;
            control_register_canctrl_value.LOOPBACK_MODE = 0x40;
            control_register_canctrl_value.LISTEN_ONLY_MODE = 0x60;
            control_register_canctrl_value.CONFIGURATION_MODE = 0x80;

            // Set values for message identifier
            register_txb0sidl_value.identifier_X = 0x00;
            register_txb0sidl_value.identifier_Y = 0x00;
            register_txb0sidl_value.identifier_Z = 0x00;
            register_txb0sidh_value.identifier_X = 0x01;
            register_txb0sidh_value.identifier_Y = 0x02;
            register_txb0sidh_value.identifier_Z = 0x03;

            // Set values for message size
            MessageSizeAdxl = 0x06;

            // Set addresss for tx buffer 0
            rEGISTER_TXB0Dx[0] = rEGISTER_TXB0D0;
            rEGISTER_TXB0Dx[1] = rEGISTER_TXB0D1;
            rEGISTER_TXB0Dx[2] = rEGISTER_TXB0D2;
            rEGISTER_TXB0Dx[3] = rEGISTER_TXB0D3;
            rEGISTER_TXB0Dx[4] = rEGISTER_TXB0D4;
            rEGISTER_TXB0Dx[5] = rEGISTER_TXB0D5;
            rEGISTER_TXB0Dx[6] = rEGISTER_TXB0D6;
            rEGISTER_TXB0Dx[7] = rEGISTER_TXB0D7;

            // Set addresss for rx buffer 0
            rEGISTER_RXB0Dx[0] = rEGISTER_RXB0D0;
            rEGISTER_RXB0Dx[1] = rEGISTER_RXB0D1;
            rEGISTER_RXB0Dx[2] = rEGISTER_RXB0D2;
            rEGISTER_RXB0Dx[3] = rEGISTER_RXB0D3;
            rEGISTER_RXB0Dx[4] = rEGISTER_RXB0D4;
            rEGISTER_RXB0Dx[5] = rEGISTER_RXB0D5;
            rEGISTER_RXB0Dx[6] = rEGISTER_RXB0D6;
            rEGISTER_RXB0Dx[7] = rEGISTER_RXB0D7;

            // Set data for interrupt flags
            control_register_canintf_value.RESET_ALL_IF = 0x00;
            control_register_canintf_value.RESET_MERRIF = 0x7F;
            control_register_canintf_value.RESET_WAKIF = 0xBF;
            control_register_canintf_value.RESET_ERRIF = 0xDF;
            control_register_canintf_value.RESET_TX2IF = 0xEF;
            control_register_canintf_value.RESET_TX1IF = 0xF7;
            control_register_canintf_value.RESET_TX0IF = 0xFB;
            control_register_canintf_value.RESET_RX1IF = 0xFD;
            control_register_canintf_value.RESET_RX0IF = 0xFE;
        }

        public byte CONTROL_REGISTER_BFPCTRL
        {
            get
            {
                return cONTROL_REGISTER_BFPCTRL;
            }

        }

        public byte CONTROL_REGISTER_TXRTSCTRL
        {
            get
            {
                return cONTROL_REGISTER_TXRTSCTRL;
            }

        }

        public byte CONTROL_REGISTER_CANSTAT
        {
            get
            {
                return cONTROL_REGISTER_CANSTAT;
            }

            set
            {
                cONTROL_REGISTER_CANSTAT = value;
            }
        }

        public byte CONTROL_REGISTER_CANCTRL
        {
            get
            {
                return cONTROL_REGISTER_CANCTRL;
            }

            set
            {
                cONTROL_REGISTER_CANCTRL = value;
            }
        }

        public byte CONTROL_REGISTER_TEC
        {
            get
            {
                return cONTROL_REGISTER_TEC;
            }

            set
            {
                cONTROL_REGISTER_TEC = value;
            }
        }

        public byte CONTROL_REGISTER_REGISTER_REC
        {
            get
            {
                return cONTROL_REGISTER_REGISTER_REC;
            }

            set
            {
                cONTROL_REGISTER_REGISTER_REC = value;
            }
        }

        public byte CONTROL_REGISTER_CNF3
        {
            get
            {
                return cONTROL_REGISTER_CNF3;
            }

            set
            {
                cONTROL_REGISTER_CNF3 = value;
            }
        }

        public byte CONTROL_REGISTER_CNF2
        {
            get
            {
                return cONTROL_REGISTER_CNF2;
            }

            set
            {
                cONTROL_REGISTER_CNF2 = value;
            }
        }

        public byte CONTROL_REGISTER_CNF1
        {
            get
            {
                return cONTROL_REGISTER_CNF1;
            }

            set
            {
                cONTROL_REGISTER_CNF1 = value;
            }
        }

        public byte CONTROL_REGISTER_CANINTE
        {
            get
            {
                return cONTROL_REGISTER_CANINTE;
            }

            set
            {
                cONTROL_REGISTER_CANINTE = value;
            }
        }

        public byte CONTROL_REGISTER_CANINTF
        {
            get
            {
                return cONTROL_REGISTER_CANINTF;
            }

            set
            {
                cONTROL_REGISTER_CANINTF = value;
            }
        }

        public byte CONTROL_REGISTER_EFLG
        {
            get
            {
                return cONTROL_REGISTER_EFLG;
            }

            set
            {
                cONTROL_REGISTER_EFLG = value;
            }
        }

        public byte CONTROL_REGISTER_TXB0CTRL
        {
            get
            {
                return cONTROL_REGISTER_TXB0CTRL;
            }

            set
            {
                cONTROL_REGISTER_TXB0CTRL = value;
            }
        }

        public byte CONTROL_REGISTER_TXB1CTRL
        {
            get
            {
                return cONTROL_REGISTER_TXB1CTRL;
            }

            set
            {
                cONTROL_REGISTER_TXB1CTRL = value;
            }
        }

        public byte CONTROL_REGISTER_TXB2CTRL
        {
            get
            {
                return cONTROL_REGISTER_TXB2CTRL;
            }

            set
            {
                cONTROL_REGISTER_TXB2CTRL = value;
            }
        }

        public byte CONTROL_REGISTER_RXB0CTRL
        {
            get
            {
                return cONTROL_REGISTER_RXB0CTRL;
            }

            set
            {
                cONTROL_REGISTER_RXB0CTRL = value;
            }
        }

        public byte CONTROL_REGISTER_RXB1CTRL
        {
            get
            {
                return cONTROL_REGISTER_RXB1CTRL;
            }

            set
            {
                cONTROL_REGISTER_RXB1CTRL = value;
            }
        }

        public byte SPI_INSTRUCTION_RESET
        {
            get
            {
                return sPI_INSTRUCTION_RESET;
            }

            set
            {
                sPI_INSTRUCTION_RESET = value;
            }
        }

        public byte SPI_INSTRUCTION_READ
        {
            get
            {
                return sPI_INSTRUCTION_READ;
            }

            set
            {
                sPI_INSTRUCTION_READ = value;
            }
        }

        public byte SPI_INSTRUCTION_WRITE
        {
            get
            {
                return sPI_INSTRUCTION_WRITE;
            }

            set
            {
                sPI_INSTRUCTION_WRITE = value;
            }
        }

        public byte SPI_INSTRUCTION_READ_STATUS
        {
            get
            {
                return sPI_INSTRUCTION_READ_STATUS;
            }

            set
            {
                sPI_INSTRUCTION_READ_STATUS = value;
            }
        }

        public byte SPI_INSTRUCTION_RX_STATUS
        {
            get
            {
                return sPI_INSTRUCTION_RX_STATUS;
            }

            set
            {
                sPI_INSTRUCTION_RX_STATUS = value;
            }
        }

        public byte SPI_INSTRUCTION_BIT_MODIFY
        {
            get
            {
                return sPI_INSTRUCTION_BIT_MODIFY;
            }

            set
            {
                sPI_INSTRUCTION_BIT_MODIFY = value;
            }
        }

        public cONTROL_REGISTER_CNFx_VALUE CONTROL_REGISTER_CNFx_VALUE
        {
            get
            {
                return control_register_cnfx_value;
            }

            set
            {
                control_register_cnfx_value = value;
            }
        }

        public cONTROL_REGISTER_CANSTAT_VALUE CONTROL_REGISTER_CANSTAT_VALUE
        {
            get
            {
                return control_register_canstat_value;
            }

            set
            {
                control_register_canstat_value = value;
            }
        }





        public cONTROL_REGISTER_CANCTRL_VALUE CONTROL_REGISTER_CANCTRL_VALUE
        {
            get
            {
                return control_register_canctrl_value;
            }

            set
            {
                control_register_canctrl_value = value;
            }
        }

        public byte REGISTER_TXB0SIDH
        {
            get
            {
                return rEGISTER_TXB0SIDH;
            }

            set
            {
                rEGISTER_TXB0SIDH = value;
            }
        }

        public byte REGISTER_TXB0SIDL
        {
            get
            {
                return rEGISTER_TXB0SIDL;
            }

            set
            {
                rEGISTER_TXB0SIDL = value;
            }
        }

        public byte REGISTER_TXB0DLC
        {
            get
            {
                return rEGISTER_TXB0DLC;
            }

            set
            {
                rEGISTER_TXB0DLC = value;
            }
        }

        public rEGISTER_TXB0SIDL_VALUE REGISTER_TXB0SIDL_VALUE
        {
            get
            {
                return register_txb0sidl_value;
            }

            set
            {
                register_txb0sidl_value = value;
            }
        }

        public rEGISTER_TXB0SIDH_VALUE REGISTER_TXB0SIDH_VALUE
        {
            get
            {
                return register_txb0sidh_value;
            }

            set
            {
                register_txb0sidh_value = value;
            }
        }

        public byte SPI_INSTRUCTION_RTS_BUFFER0
        {
            get
            {
                return sPI_INSTRUCTION_RTS_BUFFER0;
            }

            set
            {
                sPI_INSTRUCTION_RTS_BUFFER0 = value;
            }
        }

        public byte SPI_INSTRUCTION_RTS_BUFFER1
        {
            get
            {
                return sPI_INSTRUCTION_RTS_BUFFER1;
            }

            set
            {
                sPI_INSTRUCTION_RTS_BUFFER1 = value;
            }
        }

        public byte SPI_INSTRUCTION_RTS_BUFFER2
        {
            get
            {
                return sPI_INSTRUCTION_RTS_BUFFER2;
            }

            set
            {
                sPI_INSTRUCTION_RTS_BUFFER2 = value;
            }
        }

        public byte[] REGISTER_TXB0Dx
        {
            get
            {
                return rEGISTER_TXB0Dx;
            }

            set
            {
                rEGISTER_TXB0Dx = value;
            }
        }

        public byte[] REGISTER_RXB0Dx
        {
            get
            {
                return rEGISTER_RXB0Dx;
            }

            set
            {
                rEGISTER_RXB0Dx = value;
            }
        }

        public byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_ID
        {
            get
            {
                return sPI_INSTRUCTION_LOAD_TX_BUFFER0_ID;
            }

            set
            {
                sPI_INSTRUCTION_LOAD_TX_BUFFER0_ID = value;
            }
        }

        public byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA
        {
            get
            {
                return sPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA;
            }

            set
            {
                sPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA = value;
            }
        }

        public byte SPI_INSTRUCTION_READ_RX_BUFFER0_SIDH
        {
            get
            {
                return sPI_INSTRUCTION_READ_RX_BUFFER0_SIDH;
            }

            set
            {
                sPI_INSTRUCTION_READ_RX_BUFFER0_SIDH = value;
            }
        }

        public byte SPI_INSTRUCTION_READ_RX_BUFFER0_D0
        {
            get
            {
                return sPI_INSTRUCTION_READ_RX_BUFFER0_D0;
            }

            set
            {
                sPI_INSTRUCTION_READ_RX_BUFFER0_D0 = value;
            }
        }

        public cONTROL_REGISTER_CANINTF_VALUE CONTROL_REGISTER_CANINTF_VALUE
        {
            get
            {
                return control_register_canintf_value;
            }

            set
            {
                control_register_canintf_value = value;
            }
        }

        public byte MessageSizeAdxl
        {
            get
            {
                return messageSizeAdxl;
            }

            set
            {
                messageSizeAdxl = value;
            }
        }
    }
}
