using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CanTest
{
    class Logic_Mcp2515_Sender
    {
        private MCP2515 mcp2515;
        private byte[] address_TXB0Dm = new byte[8]; // Transmit register 0/2 (3 at all) and byte 0/7 (8 at all)
        private GlobalDataSet globalDataSet;
        private Data_MCP2515_Sender data_MCP2515_Sender;

        public Logic_Mcp2515_Sender(GlobalDataSet globalDataSet)
        {
            this.globalDataSet = globalDataSet;
            mcp2515 = new MCP2515();
            data_MCP2515_Sender = new Data_MCP2515_Sender();
        }

        public async void init_mcp2515_sender_task()
        {
            await Task.Run(() => init_mcp2515_sender());
        }

        public void init_mcp2515_sender()
        {
            while (globalDataSet.Spi_not_initialized)
            {
                // Wait until spi is ready
            }

            // Reset chip to set in operation mode
            mcp2515_execute_reset_command();

            // Configure bit timing
            mcp2515_configureCanBus();

            // Configure interrupts
            mcp2515_configureInterrupts();

            // Configure bit masks and filters that we can receive everything 
            mcp2515_configureMasksFilters();

            // Set device to normal mode
            mcp2515_switchMode(mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.NORMAL_MODE, mcp2515.CONTROL_REGISTER_CANCTRL_VALUE.NORMAL_MODE);
        }

        private void mcp2515_configureCanBus()
        {
            // Configure bit timing
            Debug.Write("Configure bit timing for sender" + "\n");
            byte[] spiMessage = new byte[2];

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF1;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF1;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF2;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF2;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF3;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF3;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);
        }

        public void mcp2515_execute_reset_command()
        {
            // Reset chip to get initial condition and wait for operation mode state bit
            Debug.Write("Reset chip sender" + "\n");
            byte[] returnMessage = new byte[1];

            globalDataSet.writeSimpleCommandSpi(mcp2515.SPI_INSTRUCTION_RESET, globalDataSet.MCP2515_PIN_CS_SENDER);

            // Read the register value
            byte actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_SENDER);
            while (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE != (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE & actualMode))
            {
                actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_SENDER);
            }
            Debug.Write("Switch sender to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        public void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch)
        {

            // Reset chip to get initial condition and wait for operation mode state bit
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANCTRL, modeToSwitch };
            byte[] returnMessage = new byte[1];


            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            // Read the register value
            byte actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_SENDER);
            while (modeToCheck != (modeToCheck & actualMode))
            {
                actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_SENDER);
            }
            Debug.Write("Switch sender to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        private void mcp2515_configureMasksFilters()
        {
            Debug.Write("Configure masks and filters for sender" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_TXB0CTRL, data_MCP2515_Sender.CONTROL_REGISTER_TXB0CTRL_VALUE.TXB0CTRL };

            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);
        }

        private void 
            mcp2515_configureInterrupts()
        {
            Debug.Write("Configure interrupts for sender" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANINTE, data_MCP2515_Sender.CONTROL_REGISTER_CANINTE_VALUE_SENDER.INTE };

            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);
        }

        public void mcp2515_execute_rts_command(int bufferId)
        {
            byte[] spiMessage = new byte[1];
            switch (bufferId)
            {
                case 0:
                    spiMessage[0] = mcp2515.SPI_INSTRUCTION_RTS_BUFFER0;
                    break;
                case 1:
                    spiMessage[0] = mcp2515.SPI_INSTRUCTION_RTS_BUFFER1;
                    break;
                case 2:
                    spiMessage[0] = mcp2515.SPI_INSTRUCTION_RTS_BUFFER2;
                    break;
                default:
                    break;
            }

            globalDataSet.writeSimpleCommandSpi(spiMessage[0], globalDataSet.MCP2515_PIN_CS_SENDER);
        }

        public void mcp2515_load_tx_buffer0(byte byteId, byte data)
        {
            // Send message to mcp2515 tx buffer
            Debug.Write("Load tx buffer 0 at byte " + byteId.ToString() + "\n");
            byte[] spiMessage = new byte[2];

            // Set the message identifier to 10000000000 and extended identifier bit to 0
            spiMessage[0] = mcp2515.REGISTER_TXB0SIDL;
            spiMessage[1] = mcp2515.REGISTER_TXB0SIDL_VALUE.identifier_X;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            spiMessage[0] = mcp2515.REGISTER_TXB0SIDH;
            spiMessage[1] = mcp2515.REGISTER_TXB0SIDH_VALUE.identifier_X;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            // Set data length and set rtr bit to zero (no remote request)
            spiMessage[0] = mcp2515.REGISTER_TXB0DLC;
            spiMessage[1] = mcp2515.MessageSizeAdxl;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            // Set data to tx buffer 0
            spiMessage[0] = byteId;
            spiMessage[1] = data;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_SENDER);

            // Send message
            mcp2515_execute_rts_command(0);
        }
    }
}
