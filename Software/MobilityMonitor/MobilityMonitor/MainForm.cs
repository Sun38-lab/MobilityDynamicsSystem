using ScottPlot;
using ScottPlot.Plottables;
using System.IO.Ports;
using Timer = System.Windows.Forms.Timer;

namespace MobilityMonitor
{
    public partial class MainForm : Form
    {
        private SerialPort serialPort;
        private string serialBuffer = "";

        // グラフ描画用のデータバッファ
        private DataLogger loggerAccAngle; //加速度からの角度
        private DataLogger loggerGyroAngle; //角速度からの角度
        private DataLogger loggerCompRollAngle; //補正後角ロール角
        private DataLogger loggerCompPitchAngle; //補正後のピッチ角

        // 受信したデータ数保持
        private int dataPointCount = 0;

        public MainForm()
        {
            InitializeComponent();
            SetupPlot();
            SetSerialPort();
        }

        #region 準備
        private void SetupPlot()
        {
            loggerAccAngle = formsPlot1.Plot.Add.DataLogger();
            loggerGyroAngle = formsPlot1.Plot.Add.DataLogger();
            loggerCompRollAngle = formsPlot1.Plot.Add.DataLogger();
            loggerCompPitchAngle = formsPlot1.Plot.Add.DataLogger();

            loggerAccAngle.ManageAxisLimits = false;
            loggerGyroAngle.ManageAxisLimits = false;
            loggerCompRollAngle.ManageAxisLimits = false;
            loggerCompPitchAngle.ManageAxisLimits = false;

            loggerAccAngle.Color = Colors.LightBlue; // 加速度（水色）
            loggerGyroAngle.Color = Colors.Orange;   // ジャイロ（オレンジ）
            loggerCompRollAngle.Color = Colors.Green; // ロール角
            loggerCompPitchAngle.Color = Colors.Purple; // ピッチ角

            loggerCompPitchAngle.LineWidth = 2; // ピッチ角の線を太く

            loggerCompPitchAngle.LegendText = "Pitch(deg)";
            loggerGyroAngle.LegendText = "Gyro(dps)";

            formsPlot1.Plot.ShowLegend();

            formsPlot1.Plot.Title("MobilityDynamicsSystem - Real-time Data");
            formsPlot1.Plot.Axes.Bottom.Label.Text = "Time";

            formsPlot1.Plot.Axes.SetLimitsY(-55, 55);

            Timer renderTimer = new Timer();
            renderTimer.Interval = 50;
            renderTimer.Tick += (s, e) =>
            {
                if (dataPointCount > 0)
                {
                    double currentX = dataPointCount;
                    double windowSize = 500; // 5秒分（100Hzのため）

                    double minX = currentX > windowSize ? currentX - windowSize : 0;

                    // X軸を「最新の5秒分」にスライド
                    formsPlot1.Plot.Axes.SetLimitsX(minX, currentX);

                    // Y軸を物理的な角度（例：-55度〜55度）に強制固定してオートスケールを無効化
                    formsPlot1.Plot.Axes.SetLimitsY(-55, 55);
                }
                formsPlot1.Refresh();
            };
            renderTimer.Start();
        }

        private void SetSerialPort()
        {
            serialPort = new SerialPort("COM8", 115200, Parity.None, 8, StopBits.One);
            serialPort.ReadTimeout = 50; // 読み込みの最大待ち時間を50msに設定
            serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
            try
            {
                serialPort.Open();
            }
            catch (Exception ex)
            {
                MessageBox.Show("シリアルポートが開けませんでした：" + ex.Message);
            }

        }
        #endregion

        #region 受信処理
        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            try
            {
                // 今届いているデータをすべてバッファに継ぎ足す
                serialBuffer += sp.ReadExisting();

                // バッファの中に改行コード（\n）が含まれている間、ループし続ける
                int newLineIndex;
                while ((newLineIndex = serialBuffer.IndexOf('\n')) != -1)
                {
                    // 改行までを1行のデータとして切り出し、余分な空白・改行（\rなど）をトリムする
                    string line = serialBuffer.Substring(0, newLineIndex).Trim();

                    // 処理済みの部分をバッファから消し、残りを次回に持ち越す
                    serialBuffer = serialBuffer.Substring(newLineIndex + 1);
                    System.Diagnostics.Debug.WriteLine(line);

                    // ------------------------------------------------
                    //  パース処理
                    // ------------------------------------------------
                    if (line.StartsWith("$MPU"))
                    {
                        string[] parts = line.Split(',');
                        if (parts.Length == 11)
                        {
                            // 8列目(parts[7])と9列目(parts[8])の角度データを追加
                            if (double.TryParse(parts[7], out double accAngle) &&
                                double.TryParse(parts[8], out double gyroAngle))
                            {
                                loggerAccAngle.Add(accAngle);
                                loggerGyroAngle.Add(gyroAngle);
                            }

                            // 10列目(parts[9])と11列目(parts[10])の補正後データを追加
                            if (double.TryParse(parts[9], out double compRollAngle) &&
                                double.TryParse(parts[10], out double compPitchAngle))
                            {
                                loggerCompRollAngle.Add(compRollAngle);    // 補正後ロール（緑）
                                loggerCompPitchAngle.Add(compPitchAngle);  // 補正後ピッチ（紫）

                                dataPointCount++;
                            }
                        }
                    }
                    else if (line.StartsWith("DEBUG"))
                    {
                        // 例: DEBUG,Kp:45.00,Kd:-2.00,Pitch:-16.57,Err:16.57,Gyro:-51.28,Spd:3000.00,PWM:918
                        string[] parts = line.Split(',');
                        if (parts.Length >= 8)
                        {
                            try
                            {
                                // "Pitch:-16.57" を ':' で分割して数値部分を取得
                                string pitchStr = parts[3].Split(':')[1];
                                // "Gyro:-51.28" を ':' で分割して数値部分を取得
                                string gyroStr = parts[5].Split(':')[1];

                                if (double.TryParse(pitchStr, out double pitch) &&
                                    double.TryParse(gyroStr, out double gyro))
                                {
                                    // 紫色のラインにピッチ角を描画
                                    loggerCompPitchAngle.Add(pitch);

                                    // オレンジ色のラインにジャイロ角速度を描画（プルプル監視用）
                                    loggerGyroAngle.Add(gyro);

                                    dataPointCount++;
                                }
                            }
                            catch (Exception)
                            {
                                // 分割エラー時は無視
                            }
                        }
                    }
                }
            }
            catch (Exception)
            {
                // 念のためバッファが異常に膨れ上がったらリセットする安全装置
                if (serialBuffer.Length > 10000) serialBuffer = "";
            }
        }
        #endregion

        #region ボタンクリックイベント
        private void btnSendKp_Click(object sender, EventArgs e)
        {
            // NumericUpDownの値を文字列（小数点以下2桁）に変換しコマンド作成
            string cmd = $"KP:{numKp.Value:F2}";
            SendCommandToSTM32(cmd);
        }

        private void btnSendKi_Click(object sender, EventArgs e)
        {
            string cmd = $"KI:{numKi.Value:F2}";
            SendCommandToSTM32(cmd);
        }

        private void btnSendKd_Click(object sender, EventArgs e)
        {
            string cmd = $"KD:{numKd.Value:F2}";
            SendCommandToSTM32(cmd);
        }

        private void btnEmergencyStop_Click(object sender, EventArgs e)
        {
            SendCommandToSTM32("STOP");
        }

        private void btnTarget15_Click(object sender, EventArgs e)
        {
            SendCommandToSTM32("TARGET:15.0");
        }

        private void btnTarget0_Click(object sender, EventArgs e)
        {
            SendCommandToSTM32("TARGET:0.0");
        }

        // フォーム閉じた際の処理
        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
        #endregion

        #region メソッド
        /// <summary>
        /// コマンドをシリアル通信で送信
        /// </summary>
        private void SendCommandToSTM32(string command)
        {
            // シリアルポートが開いているか確認
            if (serialPort != null && serialPort.IsOpen)
            {
                try
                {
                    serialPort.Write(command + "\n");
                }
                catch (Exception ex)
                {
                    MessageBox.Show("送信エラー: " + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else
            {
                MessageBox.Show("シリアルポートが開いていません。", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }
        #endregion
    }
}
