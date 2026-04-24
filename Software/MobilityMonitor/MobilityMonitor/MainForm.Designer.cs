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
            groupTargetAngle = new GroupBox();
            label3 = new Label();
            label2 = new Label();
            label1 = new Label();
            btnTarget0 = new Button();
            btnTarget15 = new Button();
            btnExportCSV = new Button();
            groupBox1 = new GroupBox();
            label4 = new Label();
            label5 = new Label();
            label6 = new Label();
            groupBox2 = new GroupBox();
            label7 = new Label();
            label8 = new Label();
            label9 = new Label();
            statusStrip1 = new StatusStrip();
            groupBoxGain.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)numKd).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numKi).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numKp).BeginInit();
            groupTargetAngle.SuspendLayout();
            groupBox1.SuspendLayout();
            groupBox2.SuspendLayout();
            SuspendLayout();
            // 
            // formsPlot1
            // 
            formsPlot1.DisplayScale = 1F;
            formsPlot1.Location = new Point(375, 66);
            formsPlot1.Name = "formsPlot1";
            formsPlot1.Size = new Size(833, 468);
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
            groupBoxGain.ForeColor = Color.FromArgb(241, 241, 241);
            groupBoxGain.Location = new Point(35, 49);
            groupBoxGain.Name = "groupBoxGain";
            groupBoxGain.Size = new Size(301, 169);
            groupBoxGain.TabIndex = 1;
            groupBoxGain.TabStop = false;
            groupBoxGain.Text = "PID Gain Control";
            // 
            // btnSendKd
            // 
            btnSendKd.FlatStyle = FlatStyle.Flat;
            btnSendKd.ForeColor = Color.FromArgb(241, 241, 241);
            btnSendKd.Location = new Point(194, 127);
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
            numKd.Location = new Point(51, 127);
            numKd.Minimum = new decimal(new int[] { 10, 0, 0, int.MinValue });
            numKd.Name = "numKd";
            numKd.Size = new Size(120, 23);
            numKd.TabIndex = 7;
            // 
            // labelKd
            // 
            labelKd.AutoSize = true;
            labelKd.ForeColor = Color.FromArgb(241, 241, 241);
            labelKd.Location = new Point(21, 129);
            labelKd.Name = "labelKd";
            labelKd.Size = new Size(24, 15);
            labelKd.TabIndex = 6;
            labelKd.Text = "Kd:";
            // 
            // btnSendKi
            // 
            btnSendKi.FlatStyle = FlatStyle.Flat;
            btnSendKi.ForeColor = Color.FromArgb(241, 241, 241);
            btnSendKi.Location = new Point(194, 83);
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
            numKi.Location = new Point(51, 81);
            numKi.Name = "numKi";
            numKi.Size = new Size(120, 23);
            numKi.TabIndex = 4;
            // 
            // labelKi
            // 
            labelKi.AutoSize = true;
            labelKi.ForeColor = Color.FromArgb(241, 241, 241);
            labelKi.Location = new Point(21, 83);
            labelKi.Name = "labelKi";
            labelKi.Size = new Size(20, 15);
            labelKi.TabIndex = 3;
            labelKi.Text = "Ki:";
            // 
            // btnSendKp
            // 
            btnSendKp.FlatStyle = FlatStyle.Flat;
            btnSendKp.ForeColor = Color.FromArgb(241, 241, 241);
            btnSendKp.Location = new Point(194, 38);
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
            numKp.ForeColor = Color.Black;
            numKp.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numKp.Location = new Point(51, 36);
            numKp.Maximum = new decimal(new int[] { 300, 0, 0, 0 });
            numKp.Name = "numKp";
            numKp.Size = new Size(120, 23);
            numKp.TabIndex = 1;
            numKp.Value = new decimal(new int[] { 150, 0, 0, 0 });
            // 
            // labelKp
            // 
            labelKp.AutoSize = true;
            labelKp.ForeColor = Color.FromArgb(241, 241, 241);
            labelKp.Location = new Point(21, 38);
            labelKp.Name = "labelKp";
            labelKp.Size = new Size(24, 15);
            labelKp.TabIndex = 0;
            labelKp.Text = "Kp:";
            // 
            // btnEmergencyStop
            // 
            btnEmergencyStop.BackColor = Color.Red;
            btnEmergencyStop.FlatStyle = FlatStyle.Flat;
            btnEmergencyStop.Font = new Font("Yu Gothic UI Semibold", 9F, FontStyle.Bold, GraphicsUnit.Point, 128);
            btnEmergencyStop.ForeColor = Color.White;
            btnEmergencyStop.Location = new Point(24, 35);
            btnEmergencyStop.Name = "btnEmergencyStop";
            btnEmergencyStop.Size = new Size(118, 32);
            btnEmergencyStop.TabIndex = 9;
            btnEmergencyStop.Text = "EMERGENCY STOP";
            btnEmergencyStop.UseVisualStyleBackColor = false;
            btnEmergencyStop.Click += btnEmergencyStop_Click;
            // 
            // groupTargetAngle
            // 
            groupTargetAngle.Controls.Add(label3);
            groupTargetAngle.Controls.Add(label2);
            groupTargetAngle.Controls.Add(label1);
            groupTargetAngle.Controls.Add(btnTarget0);
            groupTargetAngle.Controls.Add(btnTarget15);
            groupTargetAngle.ForeColor = Color.FromArgb(241, 241, 241);
            groupTargetAngle.Location = new Point(35, 236);
            groupTargetAngle.Name = "groupTargetAngle";
            groupTargetAngle.Size = new Size(236, 93);
            groupTargetAngle.TabIndex = 10;
            groupTargetAngle.TabStop = false;
            groupTargetAngle.Text = "Target Angle";
            // 
            // label3
            // 
            label3.AutoSize = true;
            label3.Location = new Point(30, 44);
            label3.Name = "label3";
            label3.Size = new Size(0, 15);
            label3.TabIndex = 12;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Location = new Point(30, 121);
            label2.Name = "label2";
            label2.Size = new Size(0, 15);
            label2.TabIndex = 11;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Location = new Point(24, 59);
            label1.Name = "label1";
            label1.Size = new Size(0, 15);
            label1.TabIndex = 9;
            // 
            // btnTarget0
            // 
            btnTarget0.FlatStyle = FlatStyle.Flat;
            btnTarget0.ForeColor = Color.FromArgb(241, 241, 241);
            btnTarget0.Location = new Point(133, 40);
            btnTarget0.Name = "btnTarget0";
            btnTarget0.Size = new Size(75, 23);
            btnTarget0.TabIndex = 10;
            btnTarget0.Text = "0.0Åã";
            btnTarget0.UseVisualStyleBackColor = true;
            btnTarget0.Click += btnTarget0_Click;
            // 
            // btnTarget15
            // 
            btnTarget15.FlatStyle = FlatStyle.Flat;
            btnTarget15.ForeColor = Color.FromArgb(241, 241, 241);
            btnTarget15.Location = new Point(30, 40);
            btnTarget15.Name = "btnTarget15";
            btnTarget15.Size = new Size(75, 23);
            btnTarget15.TabIndex = 9;
            btnTarget15.Text = "15.0Åã";
            btnTarget15.UseVisualStyleBackColor = true;
            btnTarget15.Click += btnTarget15_Click;
            // 
            // btnExportCSV
            // 
            btnExportCSV.BackColor = Color.FromArgb(0, 122, 204);
            btnExportCSV.FlatStyle = FlatStyle.Flat;
            btnExportCSV.ForeColor = Color.FromArgb(241, 241, 241);
            btnExportCSV.Location = new Point(36, 34);
            btnExportCSV.Name = "btnExportCSV";
            btnExportCSV.Size = new Size(75, 34);
            btnExportCSV.TabIndex = 12;
            btnExportCSV.Text = "ExportCSV";
            btnExportCSV.UseVisualStyleBackColor = false;
            btnExportCSV.Click += btnExportCSV_Click;
            // 
            // groupBox1
            // 
            groupBox1.Controls.Add(label4);
            groupBox1.Controls.Add(btnExportCSV);
            groupBox1.Controls.Add(label5);
            groupBox1.Controls.Add(label6);
            groupBox1.ForeColor = Color.FromArgb(241, 241, 241);
            groupBox1.Location = new Point(35, 350);
            groupBox1.Name = "groupBox1";
            groupBox1.Size = new Size(171, 90);
            groupBox1.TabIndex = 13;
            groupBox1.TabStop = false;
            groupBox1.Text = "Data Logging";
            // 
            // label4
            // 
            label4.AutoSize = true;
            label4.Location = new Point(30, 44);
            label4.Name = "label4";
            label4.Size = new Size(0, 15);
            label4.TabIndex = 12;
            // 
            // label5
            // 
            label5.AutoSize = true;
            label5.Location = new Point(30, 121);
            label5.Name = "label5";
            label5.Size = new Size(0, 15);
            label5.TabIndex = 11;
            // 
            // label6
            // 
            label6.AutoSize = true;
            label6.Location = new Point(24, 59);
            label6.Name = "label6";
            label6.Size = new Size(0, 15);
            label6.TabIndex = 9;
            // 
            // groupBox2
            // 
            groupBox2.Controls.Add(label7);
            groupBox2.Controls.Add(label8);
            groupBox2.Controls.Add(btnEmergencyStop);
            groupBox2.Controls.Add(label9);
            groupBox2.ForeColor = Color.FromArgb(241, 241, 241);
            groupBox2.Location = new Point(35, 467);
            groupBox2.Name = "groupBox2";
            groupBox2.Size = new Size(171, 88);
            groupBox2.TabIndex = 14;
            groupBox2.TabStop = false;
            groupBox2.Text = "Emergency";
            // 
            // label7
            // 
            label7.AutoSize = true;
            label7.Location = new Point(30, 44);
            label7.Name = "label7";
            label7.Size = new Size(0, 15);
            label7.TabIndex = 12;
            // 
            // label8
            // 
            label8.AutoSize = true;
            label8.Location = new Point(30, 121);
            label8.Name = "label8";
            label8.Size = new Size(0, 15);
            label8.TabIndex = 11;
            // 
            // label9
            // 
            label9.AutoSize = true;
            label9.Location = new Point(24, 59);
            label9.Name = "label9";
            label9.Size = new Size(0, 15);
            label9.TabIndex = 9;
            // 
            // statusStrip1
            // 
            statusStrip1.Location = new Point(0, 581);
            statusStrip1.Name = "statusStrip1";
            statusStrip1.Size = new Size(1320, 22);
            statusStrip1.TabIndex = 15;
            statusStrip1.Text = "statusStrip1";
            // 
            // MainForm
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            BackColor = Color.FromArgb(37, 37, 38);
            ClientSize = new Size(1320, 603);
            Controls.Add(statusStrip1);
            Controls.Add(groupBox2);
            Controls.Add(groupBox1);
            Controls.Add(groupTargetAngle);
            Controls.Add(groupBoxGain);
            Controls.Add(formsPlot1);
            Name = "MainForm";
            Text = "MobilityDynamicsDashboard";
            groupBoxGain.ResumeLayout(false);
            groupBoxGain.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)numKd).EndInit();
            ((System.ComponentModel.ISupportInitialize)numKi).EndInit();
            ((System.ComponentModel.ISupportInitialize)numKp).EndInit();
            groupTargetAngle.ResumeLayout(false);
            groupTargetAngle.PerformLayout();
            groupBox1.ResumeLayout(false);
            groupBox1.PerformLayout();
            groupBox2.ResumeLayout(false);
            groupBox2.PerformLayout();
            ResumeLayout(false);
            PerformLayout();
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
        private GroupBox groupTargetAngle;
        private Label label2;
        private Label label1;
        private Button btnTarget0;
        private Button btnTarget15;
        private Button btnExportCSV;
        private Label label3;
        private GroupBox groupBox1;
        private Label label4;
        private Label label5;
        private Label label6;
        private GroupBox groupBox2;
        private Label label7;
        private Label label8;
        private Label label9;
        private StatusStrip statusStrip1;
    }
}
