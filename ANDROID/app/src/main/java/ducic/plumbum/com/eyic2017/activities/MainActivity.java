package ducic.plumbum.com.eyic2017.activities;

import android.annotation.TargetApi;
import android.app.NotificationManager;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.speech.tts.TextToSpeech;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import com.android.volley.AuthFailureError;
import com.android.volley.Request;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.StringRequest;
import com.google.firebase.iid.FirebaseInstanceId;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import ducic.plumbum.com.eyic2017.R;
import ducic.plumbum.com.eyic2017.Singleton;

import static ducic.plumbum.com.eyic2017.utils.Constants.BASE_URL_DEFAULT;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<MainActivity.java>
 * Functions: 		<onCreate, ttsUnder20, ttsUnder21, onReceive, speakStatement, initFCM, sendFCM>
 * Global Variables:	<Button convert, EditText text, private TextToSpeech tts, List<RosTopic> data, MainActivity.MyReceiver receiver>
 *
 */

public class MainActivity extends AppCompatActivity {
    Button convert;
    EditText text;
    private TextToSpeech tts;
    MainActivity.MyReceiver receiver;

   /*
    *
    * Function Name: 	<onCreate>
    * Input: 		<Bundle savedInstanceState(For saving and retrieving data)>
    * Output: 		<void>
    * Logic: 		<Equivalent to 'main' function in JAVA>
    * Example Call:		<Called automatically by OS>
    *
    */

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initFCM();
        receiver = new MainActivity.MyReceiver();
        registerReceiver(receiver, new IntentFilter("eyic"));
        convert = (Button) findViewById(R.id.button);
        text = (EditText) findViewById(R.id.text_edit);
        tts = new TextToSpeech(getApplicationContext(), new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {

            }
        });

        tts.setLanguage(Locale.US);
        convert.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                speakStatement(text.getText().toString());
            }
        });
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

    public class MyReceiver extends BroadcastReceiver {

       /*
        *
        * Function Name: 	<onReceive>
        * Input: 		<Overridden function>
        * Output: 		<void>
        * Logic: 		<Receives Broadcast messages with specific title>
        * Example Call:		<Called automatically by OS>
        *
        */

        @Override
        public void onReceive(Context context, Intent intent) {
            NotificationManager notificationManager =
                    (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);
            notificationManager.cancelAll();
            speakStatement(intent.getExtras().getString("message") + "");
//            data.add(new RosTopic(intent.getExtras().getDouble("minDistance"), intent.getExtras().getDouble("leftmost"), intent.getExtras().getDouble("rightmost"), intent.getExtras().getDouble("angleLeft"), intent.getExtras().getDouble("angleRight"), intent.getExtras().getDouble("angleMin")));
        }
    }

    /*
        *
        * Function Name: 	<onReceive>
        * Input: 		<Overridden function>
        * Output: 		<void>
        * Logic: 		<Receives Broadcast messages with specific title>
        * Example Call:		<Called automatically by OS>
        *
        */

    private void speakStatement(String text){
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            ttsGreater21(text);
        } else {
            ttsUnder20(text);
        }
    }

    /*
        *
        * Function Name: 	<initFCM>
        * Input: 		<void>
        * Output: 		<void>
        * Logic: 		<Checks if FCM token present, if not, then calls necessary functions>
        * Example Call:		<initFCM()>
        *
        */

    private void initFCM() {
        SharedPreferences sp = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor;
        if (!sp.contains("token")){
            editor = sp.edit();
            if (FirebaseInstanceId.getInstance()!=null){
                String token = FirebaseInstanceId.getInstance().getToken();
                if (token != null) {
                    Log.e(getClass().getName(), token);
                    editor.putString("token", token);
                    editor.apply();
                    sendFCM(sp.getString("token", ""));
                }
            }
        }else {
            Log.e("TOKEN", sp.getString("token", ""));
        }
    }

    /*
        *
        * Function Name: 	<sendFCM>
        * Input: 		<String token(Firebase token)>
        * Output: 		<void>
        * Logic: 		<Sends Firebase token to remote server using Volley>
        * Example Call:		<sendFCM("MyFirebaseTokenisNULL")>
        *
        */

    private void sendFCM(final String token){
        StringRequest stringRequest = new StringRequest(Request.Method.POST, BASE_URL_DEFAULT + "saveToken.php",
                new Response.Listener<String>() {
                    @Override
                    public void onResponse(String response) {
                        if (response.contentEquals("UPDATED")){
                            Log.e("TAG", "updated");
                            Toast.makeText(MainActivity.this, "Token Uploaded", Toast.LENGTH_SHORT).show();
                        }else{
                            Toast.makeText(MainActivity.this, "Please Contact Pankaj", Toast.LENGTH_SHORT).show();
                        }
                    }
                }, new Response.ErrorListener() {
            @Override
            public void onErrorResponse(VolleyError error) {
                Toast.makeText(MainActivity.this, "Notifications not working!", Toast.LENGTH_SHORT).show();
            }
        }){
            @Override
            protected Map<String, String> getParams() throws AuthFailureError {
                Map<String, String> params = new HashMap<String, String>();
                params.put("firebase_token", token);
                return params;
            }
        };
        Singleton.getInstance().addToRequestQueue(stringRequest);
    }
}
