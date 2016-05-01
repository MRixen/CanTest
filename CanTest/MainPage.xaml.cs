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
        private Timer periodicTimer;
        private const byte SPI_CHIP_SELECT_LINE = 0;
        private byte[] address_TXB0Dm = new byte[8]; // Transmit register 0/2 (3 at all) and byte 0/7 (8 at all)
        private int DELTA_T = 500;
        private byte MAX_TX_BUFFER_SIZE = 8;
        private int counter;

        private GlobalDataSet globalDataSet;

        public MainPage()
        {
            this.InitializeComponent();

            // Set data that we need to execute
            mcp2515 = new MCP2515();
            globalDataSet = new GlobalDataSet();
            
            counter = 1;

            // Inititalize raspberry pi
            init_raspberry_pi_gpio();
            init_raspberry_pi_spi();

            Task task_mcp2515 = new Task(globalDataSet.init_mcp2515_task);
            task_mcp2515.Start();
            task_mcp2515.Wait();
                   
            periodicTimer = new Timer(this.TimerCallback, null, 0, DELTA_T); // Create timer to display the state of message transmission
        }

        private void init_raspberry_pi_gpio()
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
                globalDataSet.MCP2515_PIN_CS_SENDER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_19, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.MCP2515_PIN_INTE_SENDER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_5, GpioPinDriveMode.Input);
                globalDataSet.MCP2515_PIN_CS_RECEIVER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_12, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.MCP2515_PIN_INTE_RECEIVER = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_13, GpioPinDriveMode.Input);

            }
            catch (FileLoadException ex)
            {
                Debug.Write("Exception in initGPIO: " + ex + "\n");
            }
        }

        private async void init_raspberry_pi_spi()
        {
            Debug.Write("Init SPI interface" + "\n");
            try
            {
                var settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE);
                settings.ClockFrequency = 4000000;
                settings.Mode = SpiMode.Mode0;
                string aqs = SpiDevice.GetDeviceSelector();
                var dis = await DeviceInformation.FindAllAsync(aqs);
                globalDataSet.SPIDEVICE = await SpiDevice.FromIdAsync(dis[0].Id, settings);
                if (globalDataSet.SPIDEVICE == null)
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
            globalDataSet.Spi_not_initialized = true;
            while (globalDataSet.Spi_not_initialized)
            {
                bool error = false;
                try
                {
                    globalDataSet.SPIDEVICE.Write(new byte[] { 0xFF });
                }
                catch (Exception)
                {
                    error = true;
                }
                if (!error)
                {
                    globalDataSet.Spi_not_initialized = false;
                    Debug.Write("Spi device ready" + "\n");
                }
                else
                {
                    Debug.Write("Spi device not ready" + "\n");
                }
            }
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

        private void TimerCallback(object state)
        {
            bool indicatorMode = false;

            if (globalDataSet.MCP2515_PIN_INTE_RECEIVER.Read() == GpioPinValue.Low)
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
                    indicator.Background = new SolidColorBrush(Colors.Red);
                }
                else
                {
                    indicator.Background = new SolidColorBrush(Colors.Green);
                }

            });
        }

        private void Button_Click_sendMessage(object sender, RoutedEventArgs e)
        {
            if (counter >= MAX_TX_BUFFER_SIZE)
            {
                button_sendMessage.IsEnabled = false;
            }

            globalDataSet.LOGIC_MCP2515_SENDER.mcp2515_load_tx_buffer0(mcp2515.REGISTER_TXB0Dx[counter - 1]);

            byte returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_rx_buffer0(mcp2515.REGISTER_RXB0Dx[counter - 1]);
            Debug.Write("Read from buffer 0 at byte " + mcp2515.REGISTER_TXB0Dx[counter - 1].ToString() + ": " + returnMessage.ToString() + "\n");

            counter++;
        }

    }
}
