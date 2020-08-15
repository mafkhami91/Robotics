
import tkinter as tk
import serial
import time

class MyRobot(tk.Tk):#threading.Thread, tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)

        # Open a serial connection to Roomba
        self.ser = serial.Serial(port='/dev/tty.usbserial-DN026DP1', baudrate=115200)

        
        # Passive mode
        self.ser.write('\x80'.encode())
        time.sleep(.1)
        # safe mode
        self.ser.write('\x83'.encode())
        time.sleep(.1)  
        #self.root = root
        self.title("IRobot_GUI_M2A")  
        self.config(background="bisque")

        
        self.isForward = 1
        self.right_high = 0
        self.right_low = 100
        self.left_high = 0
        self.left_low = 100

        # Background Frame and its contents
        self.bgFrame = tk.Frame(self, width=200, height=600, background='bisque')
        self.bgFrame.grid(row=0, column=1, padx=10, pady=2)

        self.circleCanvas = tk.Canvas(self.bgFrame, width=100, height=100)
        self.circleCanvas.grid(row=1, column=0, padx=10, pady=2)

        self.btnFrame = tk.Frame(self.bgFrame, width=200, height=200, background='bisque')
        self.btnFrame.grid(row=2, column=0, padx=10, pady=2)

        self.moveForwardBtn = tk.Button(self.btnFrame, text="Move Forward", command=self.moveForward)
        self.moveForwardBtn.grid(row=2, column=2, padx=20, pady=2)

        self.turnLeftBtn = tk.Button(self.btnFrame, text="Turn Left", command=self.turnLeft)
        self.turnLeftBtn.grid(row=3, column=1, padx=20, pady=2)

        self.stopBtn = tk.Button(self.btnFrame, text="Stop", command=self.stop)
        self.stopBtn.grid(row=3, column=2, padx=20, pady=2)

        self.turnRightBtn = tk.Button(self.btnFrame, text="Turn Right", command=self.turnRight)
        self.turnRightBtn.grid(row=3, column=3, padx=20, pady=2)

        self.moveBackwardBtn = tk.Button(self.btnFrame, text="Move Backward", command=self.moveBackward)
        self.moveBackwardBtn.grid(row=4, column=2, padx=20, pady=2)

        self.colorLog = tk.Text(self.bgFrame, width=30, height=10, background='white')
        self.colorLog.grid(row=5, column=0, padx=10, pady=2)


    ### Add moveLeft, moveRight, stop functions Here! ###
    def moveForward(self):
        self.isForward = 1
        self.circleCanvas.create_oval(20, 20, 80, 80, width=0, fill='green')
        self.right_high = 0; self.right_low = 100; self.left_high = 0; self.left_low = 100;
        vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
        self.ser.write(bytearray(vals))
        # ser.write('\x91\x00\x64\xFF\x00'.encode())
        self.colorLog.insert(0.0, "Move Forward\n")

    def moveBackward(self):
        self.isForward = 0
        self.circleCanvas.create_oval(20, 20, 80, 80, width=0, fill='chocolate')
        self.right_high = 255; self.right_low = 156; self.left_high = 255; self.left_low = 156;
        vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
        #vals = [145, 255, 156, 255, 156] #0 -100 0 -100
        self.ser.write(bytearray(vals))
        self.colorLog.insert(0.0, "Move Backward\n")

    def turnLeft(self):
        self.circleCanvas.create_oval(20, 20, 80, 80, width=0, fill='plum')
        if self.isForward == 0:

            self.right_high = 255; self.right_low = 156; self.left_high = 255; self.left_low = 206;
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

        else:

            self.right_high = 0; self.right_low = 100; self.left_high = 0; self.left_low = 50;
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
        self.ser.write(bytearray(vals))
        self.colorLog.insert(0.0, "Turn Left\n")


    def stop(self):
        self.circleCanvas.create_oval(20, 20, 80, 80, width=0, fill='red')

        self.right_high = 0; self.right_low = 0; self.left_high = 0; self.left_low = 0;
        vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
        self.ser.write(bytearray(vals))
        self.colorLog.insert(0.0, "Stop\n")


    # if previous state was backward state, moving to the right backward
    def turnRight(self):
        self.circleCanvas.create_oval(20, 20, 80, 80, width=0, fill='cyan')
        if self.isForward == 0:
            #vals = [145, 255, 156, 255, 206] # 0 -100 0 -50

            self.right_high = 255; self.right_low = 206; self.left_high = 255; self.left_low = 156;
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
        else:
            #vals = [145, 0, 50, 0, 100]

            self.right_high = 0; self.right_low = 50; self.left_high = 0; self.left_low = 100;
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

        self.ser.write(bytearray(vals))
        # ser.write('\x91\xFF\x9C\xFF\x9C'.encode())
        self.colorLog.insert(0.0, "Turn Right\n")



    def sensorRead(self):
        self.ser.write('\x95\x01\x07'.encode())

        x = self.ser.read()
        s = str(x[0:1])[4:6]
        # s = x[0:2]  # extract
        print(s)

        vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

        if s == '01':  # if the right bumper is pressed"
            if self.right_low > self.left_low:
                self.right_low = self.left_low
            else:
                self.left_low = self.right_low

            self.right_h = 0
            self.left_h = 0
            vals = [145, self.right_h, self.right_low, self.left_high, self.left_low]

            # rotate 90 degree counterclockwise

            self.right_high = 0; self.right_low = 156; self.left_high = 255; self.left_low = 50;
            rotate_vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

            self.ser.write(bytearray(rotate_vals))
            time.sleep(1)

  

            self.ser.write(bytearray(vals))
            self.colorLog.insert(0.0, "ouch!\n")
            self.colorLog.insert(0.0, "Turning Left\n")

        elif s == '02':  # if left bumper is pressed"
            if self.right_low < self.left_low:
                self.right_low = self.left_low
            else:
                self.left_low = self.right_low

            self.right_high = 0
            self.left_high = 0
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

            # rotate 90 degree clockwise

            self.right_high = 255; self.right_low = 50; self.left_high = 0; self.left_low = 156;
            rotate_vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
            #vals = [145, 255, 50, 0, 156] # -0 50 0 -50
            self.ser.write(bytearray(rotate_vals))
            time.sleep(1)


            self.ser.write(bytearray(vals))
            self.colorLog.insert(0.0, "ouch!\n")
            self.colorLog.insert(0.0, "Turning Right\n")

        elif s == "03":  # if both bumpers are pressed
            if self.right_low > self.left_low:
                self.right_low = self.left_low
            else:
                self.left_low = self.right_low

            self.right_high = 0
            self.left_high = 0
            vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]

            # rotate 90 degree counterclockwise

            self.right_high = 0; self.right_low = 156; self.left_high = 255; self.left_low = 50;
            rotate_vals = [145, self.right_high, self.right_low, self.left_high, self.left_low]
            #vals = [145, 0, 156, 255, 50] # 0 -100 -0 50
            self.ser.write(bytearray(rotate_vals))
            time.sleep(2.1)


            self.ser.write(bytearray(vals))
            self.colorLog.insert(0.0, "ouch!\n")
            self.colorLog.insert(0.0, "I am turning back\n")

        print(vals)
        self.after(200, self.sensorRead)
        #time.sleep(.2)

def main():

    Robot = MyRobot()
    print('10 ')
    Robot.sensorRead()
    print('11')
    Robot.mainloop()  

if __name__ == "__main__":
    main()

