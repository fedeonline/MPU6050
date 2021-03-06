package main

import (
	"fmt"
	"time"
	"math"
	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/i2c"
	"github.com/hybridgroup/gobot/platforms/gpio"
	"github.com/hybridgroup/gobot/platforms/raspi"
)

const ACCEL_SENS = 16.384
const GYRO_SENS = 16.375
const CYCLE = 1

var angle_xz float64

func direction() int {
	if angle_xz > 5 {
		return 1
	}
	if angle_xz < -5 {
		return -1
	}
	return 0
}

func ReadMpu6050(Data chan [2]i2c.ThreeDData) {
	defer close(Data)
	gbot := gobot.NewGobot()
	r := raspi.NewRaspiAdaptor("raspi")
	var pin31 = gpio.NewLedDriver(r, "led", "31")
	var pin33 = gpio.NewLedDriver(r, "led", "33")
	var pin35 = gpio.NewLedDriver(r, "led", "35")
	var pin37 = gpio.NewLedDriver(r, "led", "37")
	mpu6050 := i2c.NewMPU6050Driver(r, "mpu6050", CYCLE * time.Millisecond)

	mpu6050work := func() {
		gobot.Every(CYCLE * time.Millisecond, func() {
			//加速度,陀螺仪角速度,温度
			//fmt.Print("\r", mpu6050.Accelerometer, mpu6050.Gyroscope, mpu6050.Temperature)
			Data <- [2]i2c.ThreeDData{mpu6050.Accelerometer, mpu6050.Gyroscope}
		})
	}

	motorwork := func() {
		var index int
		pinlist := []*gpio.LedDriver{pin31, pin33, pin35, pin37}
		defer func() {
			for _,p := range pinlist{
				p.Off()
			}
		}()
		steptime := time.Millisecond * 75
		gobot.Every( steptime, func() {
			index += direction()
			if index < 0 {
				index += 4
			}else if index > 3 {
				index -= 4
			}

			pin := pinlist[index]
			pin.On()
			time.Sleep(steptime)
			pin.Off()
		})
	}

	mpu6050robot := gobot.NewRobot("mpu6050Bot",
		[]gobot.Connection{r},
		[]gobot.Device{mpu6050},
		mpu6050work,
	)
	MotorRobot := gobot.NewRobot("motorBot",
		[]gobot.Connection{r},
		[]gobot.Device{pin31, pin33, pin35, pin37},
		motorwork,
	)

	gbot.AddRobot(mpu6050robot)
	gbot.AddRobot(MotorRobot)
	gbot.Start()
}

func main() {
	Data := make(chan [2]i2c.ThreeDData)
	//defer close(Data)
	go ReadMpu6050(Data)
	var ti float64
	var sgx, sgy, sgz, ax, az, gx, gy, gz float64
	for data := range Data {
		ti += 1

		ax = float64(data[0].X)
		//		ay = float64(data[0].Y)
		az = float64(data[0].Z)
		gx = float64(data[1].X) / GYRO_SENS
		gy = float64(data[1].Y) / GYRO_SENS
		gz = float64(data[1].Z) / GYRO_SENS

		sgx += gx
		sgy += gy
		sgz += gz

		angle_xz = 0.98 * (angle_xz + gy * CYCLE / 1000) + 0.02 * math.Atan2(ax, az) * 180 / math.Pi

		//		fmt.Printf("\r%.2f  %.2f  %.2f  |  %.2f  %.2f  %.2f  |  %.2f  %.2f  %.2f  |  %d  %.2f",
		//			ax,ay,az,
		//			gx,gy,gz,
		//			sgx/ti,sgy/ti,sgz/ti,
		//			int(ti),angle_xz)
		//fmt.Printf("\r %3.2f  %3.2f  %3.2f   %2d",
		//	math.Atan2(ax, az) * 180 / math.Pi, angle_xz, gy * CYCLE / 1000,direction(),
		//)
	}
}

/*
核心公式：

angle = (0.98)*(angle + gyro * dt) + (0.02)*(x_acc);

angle 姿态角
gyro  角速度
x_acc 加速度计算出来的姿态角

0.98，0.02 系数，和为1，与滤波的要求有关


深入了解角度互补滤波

angle = (0.98) * (angle + gyro*dt) + (0.02) * (x_acc);

可能你会问，为什么要在 (angle + gyro*dt) 这里直接加上角速度积分。我就不告诉你了，直接说会显得你智商好低的。
如果这里滤波器的循环频率时每秒100次(亦即循环周期 = 0.01s)，那么上述公式的时间常数就是：
tau = (a * dt)/(1-a) = ( 0.98*0.01s )/0.02s = 0.49s

这个时间常数确定了陀螺仪和加速度计的信任边界。信号时间周期低于半秒时，陀螺仪占主导地位，加速度计的噪声除掉；信号时间周期大于半秒时，加速度计的角度平均值就占据主导地位，有温漂的陀螺仪可以站一边去。
*/