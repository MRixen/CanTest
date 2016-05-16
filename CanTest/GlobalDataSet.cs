using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;

namespace CanTest
{
    class GlobalDataSet
    {
        private bool spi_not_initialized = true;
        private GpioPin mCP2515_PIN_CS_SENDER, mCP2515_PIN_INTE_SENDER;
        private GpioPin mCP2515_PIN_CS_RECEIVER, mCP2515_PIN_INTE_RECEIVER;
        private GpioPin CS_PIN_SENSOR_ADXL, sTART_PIN_OUT, aRDUINO_TEST_PIN;
        private MCP2515 mcp2515;
        private SpiDevice spiDevice;
        private Logic_Mcp2515_Sender logic_Mcp2515_Sender;
        private Logic_Mcp2515_Receiver logic_Mcp2515_Receiver;

        public GlobalDataSet()
        {
            mcp2515 = new MCP2515();
            logic_Mcp2515_Sender = new Logic_Mcp2515_Sender(this);
            logic_Mcp2515_Receiver = new Logic_Mcp2515_Receiver(this);
        }

        public async void init_mcp2515_task()
        {
            await Task.Run(() => init_mcp2515());
        }

        private void init_mcp2515()
        {
            //logic_Mcp2515_Sender.init_mcp2515_sender();
            logic_Mcp2515_Receiver.init_mcp2515_receiver();
            //init_adxl_sensor();
        }

        // FOR TESTING ONLY - AFTER FINISH TESTS REMOVE THIS
        private void init_adxl_sensor()
        {
            while (Spi_not_initialized)
            {
                // Wait until spi is ready
            }
            byte ACCEL_REG_POWER_CONTROL = 0x2D;  /* Address of the Power Control register                */
            byte ACCEL_REG_DATA_FORMAT = 0x31;    /* Address of the Data Format register                  */

            byte[] WriteBuf_DataFormat = new byte[] { ACCEL_REG_DATA_FORMAT, 0x01 };        /* 0x01 sets range to +- 4Gs                         */
            byte[] WriteBuf_PowerControl = new byte[] { ACCEL_REG_POWER_CONTROL, 0x08 };    /* 0x08 puts the accelerometer into measurement mode */

            Debug.Write("Start Sensor init \n");

            CS_PIN_SENSOR_ADXL1.Write(GpioPinValue.Low);
            SPIDEVICE.Write(WriteBuf_DataFormat);
            SPIDEVICE.Write(WriteBuf_PowerControl);
            CS_PIN_SENSOR_ADXL1.Write(GpioPinValue.High);
        }
        //-------------------------

        public bool Spi_not_initialized
        {
            get
            {
                return spi_not_initialized;
            }

            set
            {
                spi_not_initialized = value;
            }
        }

        public GpioPin MCP2515_PIN_CS_SENDER
        {
            get
            {
                return mCP2515_PIN_CS_SENDER;
            }

            set
            {
                mCP2515_PIN_CS_SENDER = value;
            }
        }

        public GpioPin MCP2515_PIN_INTE_SENDER
        {
            get
            {
                return mCP2515_PIN_INTE_SENDER;
            }

            set
            {
                mCP2515_PIN_INTE_SENDER = value;
            }
        }

        public GpioPin MCP2515_PIN_CS_RECEIVER
        {
            get
            {
                return mCP2515_PIN_CS_RECEIVER;
            }

            set
            {
                mCP2515_PIN_CS_RECEIVER = value;
            }
        }

        public GpioPin MCP2515_PIN_INTE_RECEIVER
        {
            get
            {
                return mCP2515_PIN_INTE_RECEIVER;
            }

            set
            {
                mCP2515_PIN_INTE_RECEIVER = value;
            }
        }

        public SpiDevice SPIDEVICE
        {
            get
            {
                return spiDevice;
            }

            set
            {
                spiDevice = value;
            }
        }

        public Logic_Mcp2515_Receiver LOGIC_MCP2515_RECEIVER
        {
            get
            {
                return logic_Mcp2515_Receiver;
            }

            set
            {
                logic_Mcp2515_Receiver = value;
            }
        }

        public Logic_Mcp2515_Sender LOGIC_MCP2515_SENDER
        {
            get
            {
                return logic_Mcp2515_Sender;
            }

            set
            {
                logic_Mcp2515_Sender = value;
            }
        }

        public GpioPin CS_PIN_SENSOR_ADXL1
        {
            get
            {
                return CS_PIN_SENSOR_ADXL;
            }

            set
            {
                CS_PIN_SENSOR_ADXL = value;
            }
        }

        public GpioPin START_PIN_OUT
        {
            get
            {
                return sTART_PIN_OUT;
            }

            set
            {
                sTART_PIN_OUT = value;
            }
        }

        public GpioPin ARDUINO_TEST_PIN
        {
            get
            {
                return aRDUINO_TEST_PIN;
            }

            set
            {
                aRDUINO_TEST_PIN = value;
            }
        }

        public void writeSimpleCommandSpi(byte command, GpioPin cs_pin)
        {
            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { command });
            cs_pin.Write(GpioPinValue.High);
        }

        public byte[] readSimpleCommandSpi(byte registerAddress, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[1];
            byte[] spiMessage = new byte[1];

            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_READ, registerAddress });
            spiDevice.Read(returnMessage);
            cs_pin.Write(GpioPinValue.High);

            return returnMessage;
        }

        public byte mcp2515_execute_read_command(byte registerToRead, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[1];
            byte[] sendMessage = new byte[1];

            // Enable device
            cs_pin.Write(GpioPinValue.Low);

            // Write spi instruction read  
            sendMessage[0] = mcp2515.SPI_INSTRUCTION_READ;
            spiDevice.Write(sendMessage);

            // Write the address of the register to read
            sendMessage[0] = registerToRead;
            spiDevice.Write(sendMessage);
            spiDevice.Read(returnMessage);

            // Disable device
            cs_pin.Write(GpioPinValue.High);

            return returnMessage[0];
        }

        public void mcp2515_execute_write_command(byte[] spiMessage, GpioPin cs_pin)
        {
            // Enable device
            cs_pin.Write(GpioPinValue.Low);

            // Write spi instruction write  
            spiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_WRITE });
            spiDevice.Write(spiMessage);
            cs_pin.Write(GpioPinValue.High);
        }
    }
}
