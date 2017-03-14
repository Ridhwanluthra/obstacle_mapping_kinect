package ducic.plumbum.com.eyic2017.activities;

import android.content.Intent;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;

import ducic.plumbum.com.eyic2017.R;
import ducic.plumbum.com.eyic2017.bluetooth.DeviceList;

/**
 * Created by pankaj on 14/3/17.
 */

public class LauncherActivity extends AppCompatActivity {
    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_chooser);
        Button wifi = (Button) findViewById(R.id.wifi);
        Button bluetooth = (Button) findViewById(R.id.bluetooth);
        wifi.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(LauncherActivity.this, MainActivity.class);
                startActivity(intent);
            }
        });
        bluetooth.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(LauncherActivity.this, DeviceList.class);
                startActivity(intent);
            }
        });
    }
}
