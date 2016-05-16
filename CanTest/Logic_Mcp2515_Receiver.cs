using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CanTest
{
    class Logic_Mcp2515_Receiver
    {
        private MCP2515 mcp2515;
        private GlobalDataSet globalDataSet;
        private Data_MCP2515_Receiver data_MCP2515_Receiver;

        public Logic_Mcp2515_Receiver(GlobalDataSet globalDataSet)
        {
            this.globalDataSet = globalDataSet;
            mcp2515 = new MCP2515();
            data_MCP2515_Receiver = new Data_MCP2515_Receiver();
        }

        public async void init_mcp2515_receiver_task()
        {
            await Task.Run(() => init_mcp2515_receiver());
        }

        public void init_mcp2515_receiver()
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
            Debug.Write("Configure bit timing for receiver" + "\n");
            byte[] spiMessage = new byte[2];

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF1;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF1;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF2;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF2;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF3;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF3;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);
        }

        public void mcp2515_execute_reset_command()
        {
            // Reset chip to get initial condition and wait for operation mode state bit
            Debug.Write("Reset chip receiver" + "\n");
            byte[] returnMessage = new byte[1];

            globalDataSet.writeSimpleCommandSpi(mcp2515.SPI_INSTRUCTION_RESET, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            // Read the register value
            byte actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_RECEIVER);
            while (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE != (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE & actualMode))
            {
                actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_RECEIVER);
            }
            Debug.Write("Switch receiver to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        public void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch)
        {

            // Reset chip to get initial condition and wait for operation mode state bit
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANCTRL, modeToSwitch };
            byte[] returnMessage = new byte[1];


            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            // Read the register value
            byte actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_RECEIVER);
            while (modeToCheck != (modeToCheck & actualMode))
            {
                actualMode = globalDataSet.mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT, globalDataSet.MCP2515_PIN_CS_RECEIVER);
            }
            Debug.Write("Switch receiver to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        private void mcp2515_configureMasksFilters()
        {
            Debug.Write("Configure masks and filters for receiver" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_RXB0CTRL, data_MCP2515_Receiver.CONTROL_REGISTER_RXB0CTRL_VALUE.RXB0CTRL };

            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);
        }

        private void mcp2515_configureInterrupts()
        {
            Debug.Write("Configure interrupts for receiver" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANINTE, data_MCP2515_Receiver.CONTROL_REGISTER_CANINTE_VALUE.INTE };

            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);
        }

        public byte mcp2515_read_rx_buffer0(byte byteId)
        {
            byte[] returnMessage = new byte[1];

            returnMessage = globalDataSet.readSimpleCommandSpi(byteId, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            // Slow down code (We need time between SPI-Commands)
            Task.Delay(-1).Wait(100);

            // Reset interrupt for buffer 0 because message is read -> Reset all interrupts
            globalDataSet.mcp2515_execute_write_command(new byte[] { mcp2515.CONTROL_REGISTER_CANINTF, mcp2515.CONTROL_REGISTER_CANINTF_VALUE.RESET_ALL_IF }, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            return returnMessage[0];
        }

        public void mcp2515_load_tx_buffer0(byte byteId, byte data)
        {
            // Send message to mcp2515 tx buffer
            Debug.Write("Load tx buffer 0 at byte " + byteId.ToString() + "\n");
            byte[] spiMessage = new byte[2];

            // Set data to tx buffer 0
            spiMessage[0] = byteId;
            spiMessage[1] = data;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            // Send message
            mcp2515_execute_rts_command(0);
        }

        public void mcp2515_init_tx_buffer0(byte messageLength, byte[] identifier)
        {
            byte[] spiMessage = new byte[2];

            // Set the message identifier to identifier[0] (low) and identifier[1] (high) and extended identifier to 0
            spiMessage[0] = mcp2515.REGISTER_TXB0SIDL;
            spiMessage[1] = identifier[0];
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            spiMessage[0] = mcp2515.REGISTER_TXB0SIDH;
            spiMessage[1] = identifier[1];
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            // Set data length and set rtr bit to zero (no remote request)
            spiMessage[0] = mcp2515.REGISTER_TXB0DLC;
            spiMessage[1] = messageLength;
            globalDataSet.mcp2515_execute_write_command(spiMessage, globalDataSet.MCP2515_PIN_CS_RECEIVER);
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

            globalDataSet.writeSimpleCommandSpi(spiMessage[0], globalDataSet.MCP2515_PIN_CS_RECEIVER);
        }

    }
}
