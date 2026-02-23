using ScottPlot;
using ScottPlot.Plottables;
using System.IO.Ports;
using Timer = System.Windows.Forms.Timer;

namespace MobilityMonitor
{
    public partial class MainForm : Form
    {
        private SerialPort serialPort;
        //グラフ描画用のデータバッファ
        private DataLogger loggerAccelX;
        private DataLogger loggerGyroX;

        public MainForm()
        {
            InitializeComponent();
            SetupPlot();
            SetSerialPort();
        }

        private void SetupPlot()
        {
            loggerAccelX = formsPlot1.Plot.Add.DataLogger();
            loggerGyroX = formsPlot1.Plot.Add.DataLogger();

            loggerAccelX.Color = Colors.Blue;
            loggerGyroX.Color = Colors.Red;

            // ジャイロ側のデータを右側のY軸(Right Axis)に紐付ける
            loggerGyroX.Axes.YAxis = formsPlot1.Plot.Axes.Right;

            // 左右のY軸ラベルを独立して設定し、色も合わせる
            formsPlot1.Plot.Axes.Left.Label.Text = "Acceleration (G)";
            formsPlot1.Plot.Axes.Left.Label.ForeColor = Colors.Blue;

            formsPlot1.Plot.Axes.Right.Label.Text = "Gyroscope (deg/s)";
            formsPlot1.Plot.Axes.Right.Label.ForeColor = Colors.Red;

            formsPlot1.Plot.Title("MobilityDynamicsSystem - Real-time Data");
            formsPlot1.Plot.Axes.Bottom.Label.Text = "Time";

            Timer renderTimer = new Timer();
            renderTimer.Interval = 50;
            renderTimer.Tick += (s, e) => formsPlot1.Refresh();
            renderTimer.Start();
        }

        private void SetSerialPort()
        {
            serialPort = new SerialPort("COM8",115200,Parity.None,8,StopBits.One);
            serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
            try
            {
                serialPort.Open();
            }
            catch(Exception ex)
            {
                MessageBox.Show("シリアルポートが開けませんでした："+ ex.Message);
            }

        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            try
            {
                string line = sp.ReadLine();

                if (line.StartsWith("$MPU"))
                {
                    string[] parts = line.Split(',');
                    if (parts.Length == 7)
                    {
                        // パース処理 (parts[1]=accel_X, ..., parts[4]=gyro_X)
                        if (double.TryParse(parts[1], out double accelX) &&
                            double.TryParse(parts[4], out double gyroX))
                        {
                            // DataLoggerにデータを追加（ScottPlot5のDataLoggerはスレッドセーフ）
                            loggerAccelX.Add(accelX);
                            loggerGyroX.Add(gyroX);
                        }
                    }
                }
            }
            catch (TimeoutException) { /* 読み込みタイムアウト時の処理 */ }
            catch (Exception) { /* パースエラーなどの処理 */ }
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
    }
}
