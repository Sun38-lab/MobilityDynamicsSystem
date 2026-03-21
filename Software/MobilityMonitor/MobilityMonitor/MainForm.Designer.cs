namespace MobilityMonitor
{
    partial class MainForm
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            formsPlot1 = new ScottPlot.WinForms.FormsPlot();
            groupBoxGain = new GroupBox();
            btnSendKd = new Button();
            numKd = new NumericUpDown();
            labelKd = new Label();
            btnSendKi = new Button();
            numKi = new NumericUpDown();
            labelKi = new Label();
            btnSendKp = new Button();
            numKp = new NumericUpDown();
            labelKp = new Label();
            btnEmergencyStop = new Button();
            groupBoxGain.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)numKd).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numKi).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numKp).BeginInit();
            SuspendLayout();
            // 
            // formsPlot1
            // 
            formsPlot1.DisplayScale = 1F;
            formsPlot1.Location = new Point(12, 72);
            formsPlot1.Name = "formsPlot1";
            formsPlot1.Size = new Size(552, 287);
            formsPlot1.TabIndex = 0;
            // 
            // groupBoxGain
            // 
            groupBoxGain.Controls.Add(btnSendKd);
            groupBoxGain.Controls.Add(numKd);
            groupBoxGain.Controls.Add(labelKd);
            groupBoxGain.Controls.Add(btnSendKi);
            groupBoxGain.Controls.Add(numKi);
            groupBoxGain.Controls.Add(labelKi);
            groupBoxGain.Controls.Add(btnSendKp);
            groupBoxGain.Controls.Add(numKp);
            groupBoxGain.Controls.Add(labelKp);
            groupBoxGain.Location = new Point(580, 83);
            groupBoxGain.Name = "groupBoxGain";
            groupBoxGain.Size = new Size(193, 253);
            groupBoxGain.TabIndex = 1;
            groupBoxGain.TabStop = false;
            groupBoxGain.Text = "PID Gain Control";
            // 
            // btnSendKd
            // 
            btnSendKd.Location = new Point(96, 205);
            btnSendKd.Name = "btnSendKd";
            btnSendKd.Size = new Size(75, 23);
            btnSendKd.TabIndex = 8;
            btnSendKd.Text = "Send Kd";
            btnSendKd.UseVisualStyleBackColor = true;
            btnSendKd.Click += btnSendKd_Click;
            // 
            // numKd
            // 
            numKd.DecimalPlaces = 2;
            numKd.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numKd.Location = new Point(51, 176);
            numKd.Name = "numKd";
            numKd.Size = new Size(120, 23);
            numKd.TabIndex = 7;
            // 
            // labelKd
            // 
            labelKd.AutoSize = true;
            labelKd.Location = new Point(21, 178);
            labelKd.Name = "labelKd";
            labelKd.Size = new Size(24, 15);
            labelKd.TabIndex = 6;
            labelKd.Text = "Kd:";
            // 
            // btnSendKi
            // 
            btnSendKi.Location = new Point(96, 135);
            btnSendKi.Name = "btnSendKi";
            btnSendKi.Size = new Size(75, 23);
            btnSendKi.TabIndex = 5;
            btnSendKi.Text = "Send Ki";
            btnSendKi.UseVisualStyleBackColor = true;
            btnSendKi.Click += btnSendKi_Click;
            // 
            // numKi
            // 
            numKi.DecimalPlaces = 2;
            numKi.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numKi.Location = new Point(51, 106);
            numKi.Name = "numKi";
            numKi.Size = new Size(120, 23);
            numKi.TabIndex = 4;
            // 
            // labelKi
            // 
            labelKi.AutoSize = true;
            labelKi.Location = new Point(21, 108);
            labelKi.Name = "labelKi";
            labelKi.Size = new Size(20, 15);
            labelKi.TabIndex = 3;
            labelKi.Text = "Ki:";
            // 
            // btnSendKp
            // 
            btnSendKp.Location = new Point(96, 65);
            btnSendKp.Name = "btnSendKp";
            btnSendKp.Size = new Size(75, 23);
            btnSendKp.TabIndex = 2;
            btnSendKp.Text = "Send Kp";
            btnSendKp.UseVisualStyleBackColor = true;
            btnSendKp.Click += btnSendKp_Click;
            // 
            // numKp
            // 
            numKp.DecimalPlaces = 2;
            numKp.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numKp.Location = new Point(51, 36);
            numKp.Name = "numKp";
            numKp.Size = new Size(120, 23);
            numKp.TabIndex = 1;
            numKp.Value = new decimal(new int[] { 1500, 0, 0, 131072 });
            // 
            // labelKp
            // 
            labelKp.AutoSize = true;
            labelKp.Location = new Point(21, 38);
            labelKp.Name = "labelKp";
            labelKp.Size = new Size(24, 15);
            labelKp.TabIndex = 0;
            labelKp.Text = "Kp:";
            // 
            // btnEmergencyStop
            // 
            btnEmergencyStop.BackColor = Color.Red;
            btnEmergencyStop.Font = new Font("Yu Gothic UI Semibold", 9F, FontStyle.Bold, GraphicsUnit.Point, 128);
            btnEmergencyStop.ForeColor = Color.White;
            btnEmergencyStop.Location = new Point(641, 361);
            btnEmergencyStop.Name = "btnEmergencyStop";
            btnEmergencyStop.Size = new Size(132, 34);
            btnEmergencyStop.TabIndex = 9;
            btnEmergencyStop.Text = "EMERGENCY STOP";
            btnEmergencyStop.UseVisualStyleBackColor = false;
            btnEmergencyStop.Click += btnEmergencyStop_Click;
            // 
            // MainForm
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(800, 450);
            Controls.Add(btnEmergencyStop);
            Controls.Add(groupBoxGain);
            Controls.Add(formsPlot1);
            Name = "MainForm";
            Text = "MobilityDynamicsDashboard";
            groupBoxGain.ResumeLayout(false);
            groupBoxGain.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)numKd).EndInit();
            ((System.ComponentModel.ISupportInitialize)numKi).EndInit();
            ((System.ComponentModel.ISupportInitialize)numKp).EndInit();
            ResumeLayout(false);
        }

        #endregion

        private ScottPlot.WinForms.FormsPlot formsPlot1;
        private GroupBox groupBoxGain;
        private Button btnSendKd;
        private NumericUpDown numKd;
        private Label labelKd;
        private Button btnSendKi;
        private NumericUpDown numKi;
        private Label labelKi;
        private Button btnSendKp;
        private NumericUpDown numKp;
        private Label labelKp;
        private Button btnEmergencyStop;
    }
}
