package ducic.plumbum.com.eyic2017.bluetooth;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Set;

import ducic.plumbum.com.eyic2017.R;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<DeviceList.java>
 * Functions: 		<onCreate, pairedDevicesList, onCreateOptionsMenu,
 *                  onOptionsItemSelected>
 * Global Variables:	<public static String EXTRA_ADDRESS, Button btnPaired,
 *                      ListView devicelist, BluetoothAdapter myBluetooth,
 *                      Set<BluetoothDevice> pairedDevices,
 *                      AdapterView.OnItemClickListener myListClickListener>
 *
 */

public class DeviceList extends AppCompatActivity {
    public static String EXTRA_ADDRESS = "device_address";
    //widgets
    Button btnPaired;
    ListView devicelist;
    //Bluetooth
    private BluetoothAdapter myBluetooth = null;
    private Set<BluetoothDevice> pairedDevices;
    private AdapterView.OnItemClickListener myListClickListener = new AdapterView
            .OnItemClickListener() {
        public void onItemClick(AdapterView<?> av, View v, int arg2, long arg3) {
            // Get the device MAC address, the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            String address = info.substring(info.length() - 17);

            // Make an intent to start next activity.
            Intent i = new Intent(DeviceList.this, NewActivity.class);

            //Change the activity.
            i.putExtra(EXTRA_ADDRESS, address); //this will be received at ledControl (class)
            // Activity
            startActivity(i);
        }
    };

    /*
    *
    * Function Name: 	<Function Name>
    * Input: 		<Inputs (or Parameters) list with description if any>
    * Output: 		<Return value with description if any>
    * Logic: 		<Description of the function performed and the logic used in the function>
    * Example Call:		<Example of how to call this function>
    *
    */

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_device_list);

        //Calling widgets
        btnPaired = (Button) findViewById(R.id.button);
        devicelist = (ListView) findViewById(R.id.listView);

        //if the device has bluetooth
        myBluetooth = BluetoothAdapter.getDefaultAdapter();

        if (myBluetooth == null) {
            //Show a mensag. that the device has no bluetooth adapter
            Toast.makeText(getApplicationContext(), "Bluetooth Device Not Available", Toast
                    .LENGTH_LONG).show();

            //finish apk
            finish();
        } else if (!myBluetooth.isEnabled()) {
            //Ask to the user turn the bluetooth on
            Intent turnBTon = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnBTon, 1);
        }

        btnPaired.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                pairedDevicesList();
            }
        });

    }

    /*
    *
    * Function Name: 	<Function Name>
    * Input: 		<Inputs (or Parameters) list with description if any>
    * Output: 		<Return value with description if any>
    * Logic: 		<Description of the function performed and the logic used in the function>
    * Example Call:		<Example of how to call this function>
    *
    */

    private void pairedDevicesList() {
        pairedDevices = myBluetooth.getBondedDevices();
        ArrayList<String> list = new ArrayList<String>();

        if (pairedDevices.size() > 0) {
            for (BluetoothDevice bt : pairedDevices) {
                list.add(bt.getName() + "\n" + bt.getAddress()); //Get the device's name and the
                // address
            }
        } else {
            Toast.makeText(getApplicationContext(), "No Paired Bluetooth Devices Found.", Toast
                    .LENGTH_LONG).show();
        }

        final ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout
                .simple_list_item_1, list);
        devicelist.setAdapter(adapter);
        devicelist.setOnItemClickListener(myListClickListener); //Method called when the device
        // from the list is clicked

    }

    /*
    *
    * Function Name: 	<Function Name>
    * Input: 		<Inputs (or Parameters) list with description if any>
    * Output: 		<Return value with description if any>
    * Logic: 		<Description of the function performed and the logic used in the function>
    * Example Call:		<Example of how to call this function>
    *
    */

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_device_list, menu);
        return true;
    }

    /*
    *
    * Function Name: 	<Function Name>
    * Input: 		<Inputs (or Parameters) list with description if any>
    * Output: 		<Return value with description if any>
    * Logic: 		<Description of the function performed and the logic used in the function>
    * Example Call:		<Example of how to call this function>
    *
    */

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
