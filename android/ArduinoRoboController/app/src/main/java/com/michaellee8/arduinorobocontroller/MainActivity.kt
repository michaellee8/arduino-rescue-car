package com.michaellee8.arduinorobocontroller

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.MotionEvent
import android.widget.Button
import android.widget.SeekBar
import android.widget.SeekBar.OnSeekBarChangeListener
import android.widget.TextView
import android.widget.Toast
import com.harrysoft.androidbluetoothserial.BluetoothManager;
import com.harrysoft.androidbluetoothserial.BluetoothSerialDevice
import com.harrysoft.androidbluetoothserial.SimpleBluetoothDeviceInterface
import io.github.controlwear.virtual.joystick.android.JoystickView


class MainActivity : AppCompatActivity() {

    fun toast(string: String) {
//        Toast.makeText(applicationContext, string, Toast.LENGTH_LONG).show()
        findViewById<TextView>(R.id.logger).text = string
    }

    var serialDevice: BluetoothSerialDevice? = null
    var deviceInterface: SimpleBluetoothDeviceInterface? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val bluetoothManager = BluetoothManager.getInstance()

        if (bluetoothManager == null) {
            Toast.makeText(applicationContext, "Bluetooth not available.", Toast.LENGTH_LONG).show()
            print("no bluetooth")
        }

        val pairedDevices = bluetoothManager.pairedDevicesList
        val mac = pairedDevices.find { it.name.contains("F6") }?.address
        if (mac == null) {
            Toast.makeText(applicationContext, "No such device.", Toast.LENGTH_LONG).show()
            print("no device")
        }

        Toast.makeText(applicationContext, "getting device", Toast.LENGTH_LONG).show()

        serialDevice = bluetoothManager.openSerialDevice(mac).blockingGet()

        deviceInterface = serialDevice?.toSimpleDeviceInterface()


        toast("got device")

        val fastBtn = findViewById<Button>(R.id.fast_btn)
        val slowBtn = findViewById<Button>(R.id.slow_btn)

        fastBtn.setOnClickListener {
            deviceInterface?.sendMessage("L")
            toast("L")
        }
        slowBtn.setOnClickListener {
            deviceInterface?.sendMessage("M")
            toast("M")
        }

        val leftBtn = findViewById<Button>(R.id.left_btn)
        val rightBtn = findViewById<Button>(R.id.right_btn)

        rightBtn.setOnTouchListener{ v, e ->
            if (e.action == MotionEvent.ACTION_DOWN){
                deviceInterface?.sendMessage("G")
                toast("G")
            }
            if (e.action == MotionEvent.ACTION_UP){
                deviceInterface?.sendMessage("z")
                toast("z")
            }

            false
        }
        leftBtn.setOnTouchListener { v, e ->
            if (e.action == MotionEvent.ACTION_DOWN){
                deviceInterface?.sendMessage("C")
                toast("C")
            }
            if (e.action == MotionEvent.ACTION_UP){
                deviceInterface?.sendMessage("z")
                toast("z")
            }
            false
        }

        val tiltBar = findViewById<SeekBar>(R.id.tilt_bar)

        tiltBar.setOnSeekBarChangeListener(object : OnSeekBarChangeListener {
            override fun onProgressChanged(seekbar: SeekBar, progress: Int, b: Boolean) {
                deviceInterface?.sendMessage(progress.toString())
                toast(progress.toString())
            }

            override fun onStartTrackingTouch(p0: SeekBar?) {

            }

            override fun onStopTrackingTouch(p0: SeekBar?) {

            }
        })

        val joystick = findViewById<JoystickView>(R.id.joystick)
        joystick.setOnMoveListener { angle, strength ->
            if (strength <= 30){
                deviceInterface?.sendMessage("z")
                return@setOnMoveListener
            }

            val angleEff = (angle + 270) % 360
            when (angleEff) {
                in 0..22 -> deviceInterface?.sendMessage("A")
                in 23..67 -> deviceInterface?.sendMessage("D")
                in 68..112 -> deviceInterface?.sendMessage("d")
                in 113..157 -> deviceInterface?.sendMessage("B")
                in 158..202 -> deviceInterface?.sendMessage("E")
                in 203..247 -> deviceInterface?.sendMessage("H")
                in 248..292 -> deviceInterface?.sendMessage("b")
                in 293..337 -> deviceInterface?.sendMessage("F")
                in 338..360 -> deviceInterface?.sendMessage("A")
            }
        }
    }


}