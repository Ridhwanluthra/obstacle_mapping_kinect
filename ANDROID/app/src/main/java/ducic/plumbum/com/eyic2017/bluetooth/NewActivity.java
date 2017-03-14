package ducic.plumbum.com.eyic2017.bluetooth;

import android.annotation.TargetApi;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.speech.tts.TextToSpeech;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Locale;
import java.util.UUID;

import ducic.plumbum.com.eyic2017.R;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<NewActivity.java>
 * Functions: 		<onCreate, createBluetoothSocket, onResume, onPause,
 *                  checkBTState, errorExit, run, write, ttsUnder20, ttsUnder21>
 * Global Variables:	<String TAG, Button btnOn, btnOff, TextView txtArduino,
 *                      Handler h, int RECIEVE_MESSAGE, BluetoothAdapter btAdapter,
 *                      BluetoothSocket btSocket, StringBuilder sb,
 *                      ConnectedThread mConnectedThread, UUID MY_UUID,
 *                      String address, TextToSpeech tts>
 *
 */

public class NewActivity extends Activity {

    private static final String TAG = "bluetooth2";

    Button btnOn, btnOff;
    TextView txtArduino;
    Handler h;

    final int RECIEVE_MESSAGE = 1;        // Status  for Handler
    private BluetoothAdapter btAdapter = null;
    private BluetoothSocket btSocket = null;
    private StringBuilder sb = new StringBuilder();

    private ConnectedThread mConnectedThread;

    // SPP UUID service
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // MAC-address of Bluetooth module (you must edit this line)
    private static String address = "00:15:FF:F2:19:5F";

    private TextToSpeech tts;

    /*
    *
    * Function Name: 	<onCreate>
    * Input: 		<Bundle savedInstanceState(For saving and retrieving data)>
    * Output: 		<void>
    * Logic: 		<Equivalent to 'main' function in JAVA>
    * Example Call:		<Called automatically by OS>
    *
    */

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Intent newint = getIntent();
        address = newint.getStringExtra(DeviceList.EXTRA_ADDRESS); //receive the address of the
        setContentView(R.layout.activity_new);
        Log.e("ADDRESS", address);

        btnOn = (Button) findViewById(R.id.btnOn);                  // button LED ON
        btnOff = (Button) findViewById(R.id.btnOff);                // button LED OFF
        txtArduino = (TextView) findViewById(R.id.data);      // for display the received data from the Arduino
        txtArduino.setMovementMethod(new ScrollingMovementMethod());

        tts = new TextToSpeech(getApplicationContext(), new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
            }
        });
        tts.setLanguage(Locale.US);

        h = new Handler() {
            public void handleMessage(android.os.Message msg) {
                switch (msg.what) {
                    case RECIEVE_MESSAGE:                                                   // if receive massage
                        byte[] readBuf = (byte[]) msg.obj;
                        String strIncom = new String(readBuf, 0, msg.arg1);                 // create string from bytes array
                        sb.append(strIncom);                                                // append string
                        int endOfLineIndex = sb.indexOf("\r\n");                            // determine the end-of-line
                        if (endOfLineIndex > 0) {                                            // if end-of-line,
                            String text = sb.substring(0, endOfLineIndex);               // extract string
                            sb.delete(0, sb.length());                                      // and clear

                            txtArduino.setText(text);            // update TextView
                            Log.e("TEXT", text);

                            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                                ttsGreater21(text);
                            } else {
                                ttsUnder20(text);
                            }
                        }
                        break;
                }
            };
        };

        btAdapter = BluetoothAdapter.getDefaultAdapter();       // get Bluetooth adapter
        checkBTState();

        btnOn.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                btnOn.setEnabled(false);
                btnOff.setEnabled(true);
                mConnectedThread.write("1");    // Send "1" via Bluetooth
                //Toast.makeText(getBaseContext(), "Turn on LED", Toast.LENGTH_SHORT).show();
            }
        });

        btnOff.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                btnOff.setEnabled(false);
                btnOn.setEnabled(true);
                mConnectedThread.write("0");    // Send "0" via Bluetooth
                //Toast.makeText(getBaseContext(), "Turn off LED", Toast.LENGTH_SHORT).show();
            }
        });
    }

    /*
    *
    * Function Name: 	<createBluetoothSocket>
    * Input: 		<BluetoothDevice device(connected bluetooth device)>
    * Output: 		<BluetoothSocket(returns socket to the connected device)>
    * Logic: 		<returns socket to the connected device>
    * Example Call:		<createBluetoothSocket(new BluetoothDevice)>
    *
    */

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
        try {
            final Method m = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord", UUID.class);
            return (BluetoothSocket) m.invoke(device, MY_UUID);
        } catch (Exception e) {
            Log.e(TAG, "Could not create Insecure RFComm Connection",e);
        }
        return  device.createRfcommSocketToServiceRecord(MY_UUID);
    }

    /*
    *
    * Function Name: 	<onOptionsItemSelected>
    * Input: 		<Overridden function>
    * Output: 		<Overridden function>
    * Logic: 		<Overridden function>
    * Example Call:		<System makes automatic calls>
    *
    */

    @Override
    public void onResume() {
        super.onResume();

        Log.d(TAG, "...onResume - try connect...");

        // Set up a pointer to the remote node using it's address.
        BluetoothDevice device = btAdapter.getRemoteDevice(address);

        // Two things are needed to make a connection:
        //   A MAC address, which we got above.
        //   A Service ID or UUID.  In this case we are using the
        //     UUID for SPP.

        try {
            btSocket = createBluetoothSocket(device);
        } catch (IOException e) {
            errorExit("Fatal Error", "In onResume() and socket create failed: " + e.getMessage() + ".");
        }

        // Discovery is resource intensive.  Make sure it isn't going on
        // when you attempt to connect and pass your message.
        btAdapter.cancelDiscovery();

        // Establish the connection.  This will block until it connects.
        Log.d(TAG, "...Connecting...");
        try {
            btSocket.connect();
            Log.d(TAG, "....Connection ok...");
        } catch (IOException e) {
            try {
                btSocket.close();
            } catch (IOException e2) {
                errorExit("Fatal Error", "In onResume() and unable to close socket during connection failure" + e2.getMessage() + ".");
            }
        }

        // Create a data stream so we can talk to server.
        Log.d(TAG, "...Create Socket...");

        mConnectedThread = new ConnectedThread(btSocket);
        mConnectedThread.start();
    }

    /*
    *
    * Function Name: 	<onOptionsItemSelected>
    * Input: 		<Overridden function>
    * Output: 		<Overridden function>
    * Logic: 		<Overridden function>
    * Example Call:		<System makes automatic calls>
    *
    */

    @Override
    public void onPause() {
        super.onPause();

        Log.d(TAG, "...In onPause()...");

        try     {
            btSocket.close();
        } catch (IOException e2) {
            errorExit("Fatal Error", "In onPause() and failed to close socket." + e2.getMessage() + ".");
        }
    }

    /*
    *
    * Function Name: 	<checkBTState>
    * Input: 		<void>
    * Output: 		<void>
    * Logic: 		<Check for Bluetooth support and then check to make sure it is turned on>
    * Example Call:		<checkBTState()>
    *
    */

    private void checkBTState() {
        // Check for Bluetooth support and then check to make sure it is turned on
        // Emulator doesn't support Bluetooth and will return null
        if(btAdapter==null) {
            errorExit("Fatal Error", "Bluetooth not support");
        } else {
            if (btAdapter.isEnabled()) {
                Log.d(TAG, "...Bluetooth ON...");
            } else {
                //Prompt user to turn on Bluetooth
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, 1);
            }
        }
    }

    /*
    *
    * Function Name: 	<errorExit>
    * Input: 		<String title(title of error), String message(Actual error message)>
    * Output: 		<void>
    * Logic: 		<Toast is generated on error occurance>
    * Example Call:		<errorExit("Error Title", "Error Message")>
    *
    */

    private void errorExit(String title, String message){
        Toast.makeText(getBaseContext(), title + " - " + message, Toast.LENGTH_LONG).show();
        finish();
    }

    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        ConnectedThread(BluetoothSocket socket) {
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException ignored) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        /*
        *
        * Function Name: 	<run>
        * Input: 		<void>
        * Output: 		<void>
        * Logic: 		<Automatically called>
        * Example Call:		<It is a thread. call Thread.execute()>
        *
        */

        public void run() {
            byte[] buffer = new byte[256];  // buffer store for the stream
            int bytes; // bytes returned from read()

            // Keep listening to the InputStream until an exception occurs
            while (true) {
                try {
                    // Read from the InputStream
                    bytes = mmInStream.read(buffer);        // Get number of bytes and message in "buffer"
                    h.obtainMessage(RECIEVE_MESSAGE, bytes, -1, buffer).sendToTarget();     // Send to message queue Handler
                } catch (IOException e) {
                    break;
                }
            }
        }

        /*
        *
        * Function Name: 	<write>
        * Input: 		<Inputs (or Parameters) list with description if any>
        * Output: 		<Return value with description if any>
        * Logic: 		<Description of the function performed and the logic used in the function>
        * Example Call:		<Example of how to call this function>
        *
        */

        /* Call this from the main activity to send data to the remote device */
        public void write(String message) {
            Log.d(TAG, "...Data to send: " + message + "...");
            byte[] msgBuffer = message.getBytes();
            try {
                mmOutStream.write(msgBuffer);
            } catch (IOException e) {
                Log.d(TAG, "...Error data send: " + e.getMessage() + "...");
            }
        }
    }

    /*
    *
    * Function Name: 	<ttsUnder20>
    * Input: 		<String text(Text to be spoken)>
    * Output: 		<void>
    * Logic: 		<Function calls the necessary Google APIs for devices with max API Level 20>
    * Example Call:		<ttsUnder20("Hello World!")>
    *
    */

    @SuppressWarnings("deprecation")
    private void ttsUnder20(String text) {
        HashMap<String, String> map = new HashMap<>();
        map.put(TextToSpeech.Engine.KEY_PARAM_UTTERANCE_ID, "MessageId");
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, map);
    }

    /*
    *
    * Function Name: 	<ttsUnder21>
    * Input: 		<String text(Text to be spoken)>
    * Output: 		<void>
    * Logic: 		<Function calls the necessary Google APIs for devices with min API Level 21>
    * Example Call:		<ttsUnder21("Hello World!")>
    *
    */

    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    private void ttsGreater21(String text) {
        String utteranceId=this.hashCode() + "";
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, utteranceId);
    }
}