using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace CanTest
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private MCP2515 mcp2515;
        private RASPBERRYPI raspberrypi;
        private Timer periodicTimer;
        private SpiDevice SpiDevice;
        private const byte SPI_CHIP_SELECT_LINE = 0;
        private GpioPin MCP2515_PIN_CS_SENDER, MCP2515_PIN_INTE_SENDER;
        private byte[] address_TXB0Dm = new byte[8]; // Transmit register 0/2 (3 at all) and byte 0/7 (8 at all)
        private int DELTA_T = 500;
        private byte MAX_TX_BUFFER_SIZE = 8;
        private int counter;

        public MainPage()
        {
            this.InitializeComponent();

            mcp2515 = new MCP2515();
            counter = 1;

            init_raspberry_pi();

        }

        private void init_raspberry_pi()
        {
            Debug.Write("Start GPIO init \n");

            var gpioController = GpioController.GetDefault();

            if (gpioController == null)
            {
                return;
            }
            try
            {
                Debug.Write("Configure pins \n");
                // Configure pins
                MCP2515_PIN_CS_SENDER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_19, GpioPinValue.High, GpioPinDriveMode.Output);
                MCP2515_PIN_INTE_SENDER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_5, GpioPinDriveMode.Input);

            }
            catch (FileLoadException ex)
            {
                Debug.Write("Exception in initGPIO: " + ex + "\n");
            }

            // Initialize spi interface
            init_spi_raspberry();
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinValue pinValue, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.Write(pinValue);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private async void init_spi_raspberry()
        {
            Debug.Write("Init SPI interface" + "\n");
            try
            {
                var settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE);
                settings.ClockFrequency = 4000000;
                settings.Mode = SpiMode.Mode0;
                string aqs = SpiDevice.GetDeviceSelector();
                var dis = await DeviceInformation.FindAllAsync(aqs);
                SpiDevice = await SpiDevice.FromIdAsync(dis[0].Id, settings);
                if (SpiDevice == null)
                {
                    Debug.Write("SPI Controller is currently in use by another application. \n");
                    return;
                }
            }
            catch (Exception ex)
            {
                Debug.Write("SPI Initialization failed. Exception: " + ex.Message + "\n");
                return;
            }

            // Send something to check that spi device is ready
            bool notReady = true;
            while (notReady)
            {
                bool error = false;
                try
                {
                    SpiDevice.Write(new byte[] { 0xFF });
                }
                catch (Exception)
                {
                    error = true;
                }
                if (!error)
                {
                    notReady = false;
                    Debug.Write("Spi device ready" + "\n");
                }
                else
                {
                    Debug.Write("Spi device not ready" + "\n");
                }
            }

            init_mcp2515_sender();
            //init_mcp2515_receiver();

            periodicTimer = new Timer(this.TimerCallback, null, 0, DELTA_T); // Create timer to display the state of interrupt pin INT (message in rx buffer)
        }

        private void init_mcp2515_sender()
        {
            // Reset chip to set in operation mode
            mcp2515_execute_reset_command();

            // Configure bit timing
            mcp2515_configureCanBus();

            // Configure interrupts
            mcp2515_configureInterrupts();

            // Configure bit masks and filters that we can receive everything 
            mcp2515_configureMasksFilters();

            // Set device to normal mode
            mcp2515_switchMode(mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.LOOPBACK_MODE, mcp2515.CONTROL_REGISTER_CANCTRL_VALUE.LOOPBACK_MODE);
        }

        private void mcp2515_configureCanBus()
        {
            // Configure bit timing
            Debug.Write("Configure bit timing" + "\n");
            byte[] spiMessage = new byte[2];

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF1;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF1;
            mcp2515_execute_write_command(spiMessage);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF2;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF2;
            mcp2515_execute_write_command(spiMessage);

            spiMessage[0] = mcp2515.CONTROL_REGISTER_CNF3;
            spiMessage[1] = mcp2515.CONTROL_REGISTER_CNFx_VALUE.CNF3;
            mcp2515_execute_write_command(spiMessage);
        }

        private void mcp2515_execute_reset_command()
        {
            // Reset chip to get initial condition and wait for operation mode state bit
            Debug.Write("Reset chip" + "\n");
            byte[] returnMessage = new byte[1];

            // Enable device
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.Low);
            SpiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_RESET });
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.High);

            // Read the register value
            byte actualMode = mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT);
            while (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE != (mcp2515.CONTROL_REGISTER_CANSTAT_VALUE.CONFIGURATION_MODE & actualMode))
            {
                actualMode = mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT);
                Debug.Write("Actual mode " + actualMode + "\n");
            }
            Debug.Write("Switch to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        private void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch)
        {

            // Reset chip to get initial condition and wait for operation mode state bit
            Debug.Write("Switch device to normal operation mode" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANCTRL, modeToSwitch };
            byte[] returnMessage = new byte[1];


            mcp2515_execute_write_command(spiMessage);

            // Read the register value
            byte actualMode = mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT);
            while (modeToCheck != (modeToCheck & actualMode))
            {
                actualMode = mcp2515_execute_read_command(mcp2515.CONTROL_REGISTER_CANSTAT);
            }
            Debug.Write("Switch to mode " + actualMode.ToString() + " successfully" + "\n");
        }

        private void mcp2515_configureMasksFilters()
        {
            Debug.Write("Configure masks and filters" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_RXB0CTRL, mcp2515.CONTROL_REGISTER_RXB0CTRL_VALUE_RECEIVER.RXB0CTRL };

            mcp2515_execute_write_command(spiMessage);
        }

        private void mcp2515_configureInterrupts()
        {
            Debug.Write("Configure interrupts for receiver" + "\n");
            byte[] spiMessage = new byte[] { mcp2515.CONTROL_REGISTER_CANINTE, mcp2515.CONTROL_REGISTER_CANINTE_VALUE_RECEIVER.INTE };

            mcp2515_execute_write_command(spiMessage);
        }

        private byte mcp2515_execute_read_command(byte registerToRead)
        {
            byte[] returnMessage = new byte[1];
            byte[] sendMessage = new byte[1];

            // Enable device
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.Low);

            // Write spi instruction read  
            sendMessage[0] = mcp2515.SPI_INSTRUCTION_READ;
            SpiDevice.Write(sendMessage);

            // Write the address of the register to read
            sendMessage[0] = registerToRead;
            SpiDevice.Write(sendMessage);
            SpiDevice.Read(returnMessage);

            // Disable device
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.High);

            return returnMessage[0];
        }

        private byte mcp2515_read_rx_buffer0(byte byteId)
        {
            byte[] returnMessage = new byte[1];
            byte[] spiMessage = new byte[1];

            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.Low);
            SpiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_READ, byteId });
            SpiDevice.Read(returnMessage);
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.High);

            // Slow down code (We need time between SPI-Commands)
            Task.Delay(-1).Wait(100);

            // Reset interrupt for buffer 0 because message is read -> Reset all interrupts
            mcp2515_execute_write_command(new byte[] {mcp2515.CONTROL_REGISTER_CANINTF, mcp2515.CONTROL_REGISTER_CANINTF_VALUE.RESET_ALL_IF});

            return returnMessage[0];
        }

        private void mcp2515_execute_write_command(byte[] spiMessage)
        {
            // Enable device
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.Low);

            // Write spi instruction write  
            SpiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_WRITE });
            SpiDevice.Write(spiMessage);
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.High);
        }

        private void mcp2515_execute_rts_command(int bufferId)
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

            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.Low);
            SpiDevice.Write(spiMessage);
            MCP2515_PIN_CS_SENDER.Write(GpioPinValue.High);
        }

        private void mcp2515_load_tx_buffer0(byte byteId)
        {
            // Send message to mcp2515 tx buffer
            Debug.Write("Load tx buffer 0 " + "at byte " + byteId.ToString() + "\n");
            byte[] spiMessage = new byte[2];

            // Set the message identifier to 10000000000 and extended identifier bit to 0
            spiMessage[0] = mcp2515.REGISTER_TXB0SIDL;
            spiMessage[1] = mcp2515.REGISTER_TXB0SIDL_VALUE.identifier_X;
            mcp2515_execute_write_command(spiMessage);

            spiMessage[0] = mcp2515.REGISTER_TXB0SIDH;
            spiMessage[1] = mcp2515.REGISTER_TXB0SIDH_VALUE.identifier_X;
            mcp2515_execute_write_command(spiMessage);

            // Set data length and set rtr bit to zero (no remote request)
            spiMessage[0] = mcp2515.REGISTER_TXB0DLC;
            spiMessage[1] = mcp2515.REGISTER_TXB0DLC_VALUE.messageSize_X;
            mcp2515_execute_write_command(spiMessage);

            // Set data to tx buffer 0
            spiMessage[0] = byteId;
            spiMessage[1] = 0xF0;
            mcp2515_execute_write_command(spiMessage);

            // Send message
            mcp2515_execute_rts_command(0);
        }

        private void TimerCallback(object state)
        {
            bool indicatorMode = false;

            if (MCP2515_PIN_INTE_SENDER.Read() == GpioPinValue.Low)
            {
                indicatorMode = true;
            }
            else
            {
                indicatorMode = false;
            }

            /* UI updates must be invoked on the UI thread */
            var task = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (indicatorMode)
                {
                    indicator.Background = new SolidColorBrush(Colors.Green);
                }
                else
                {
                    indicator.Background = new SolidColorBrush(Colors.Black);
                }

            });
        }

        private void Button_Click_sendMessage(object sender, RoutedEventArgs e)
        {
            if (counter >= MAX_TX_BUFFER_SIZE)
            {
                button_sendMessage.IsEnabled = false;
            }

            mcp2515_load_tx_buffer0(mcp2515.REGISTER_TXB0Dx[counter - 1]);

            byte returnMessage = mcp2515_read_rx_buffer0(mcp2515.REGISTER_TXB0Dx[counter - 1]);
            Debug.Write("Read from buffer 0 at byte " + mcp2515.REGISTER_TXB0Dx[counter - 1].ToString() + ": " + returnMessage.ToString() + "\n");

            counter++;
        }

    }
}
