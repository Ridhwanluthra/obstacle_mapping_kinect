package ducic.plumbum.com.eyic2017.fcm;

import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.media.RingtoneManager;
import android.net.Uri;
import android.support.v4.app.NotificationCompat;
import android.util.Log;
import android.widget.Toast;

import com.google.firebase.messaging.FirebaseMessagingService;
import com.google.firebase.messaging.RemoteMessage;

import java.util.Map;

import ducic.plumbum.com.eyic2017.activities.MainActivity;
import ducic.plumbum.com.eyic2017.R;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<MyFirebaseMessagingService.java>
 * Functions: 		<onMessageReceived, sendNotification>
 * Global Variables:	<String TAG>
 *
 */
public class MyFirebaseMessagingService extends FirebaseMessagingService {
    private static final String TAG = "MyFirebaseMsgService";

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
    public void onMessageReceived(RemoteMessage remoteMessage) {
        super.onMessageReceived(remoteMessage);

        if (remoteMessage.getData()!=null && remoteMessage.getData().containsKey("message")) {
            sendNotification(remoteMessage.getData());
            Intent intents=new Intent();
            intents.setAction("eyic");
            intents.putExtra("message", remoteMessage.getData().get("message") + "");
            Log.e(TAG, remoteMessage.getData().get("message") + "");

            getBaseContext().sendBroadcast(intents);
        }else{
            Toast.makeText(this, "Error in notifications", Toast.LENGTH_SHORT).show();
        }
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

    private void sendNotification(Map<String, String> messageBody) {
        Intent intent = new Intent(this, MainActivity.class);
        Log.e("TAG", messageBody.get("message"));
        intent.putExtra("message", messageBody.get("message"));

        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
        PendingIntent pendingIntent = PendingIntent.getActivity(this, 0, intent,
                PendingIntent.FLAG_ONE_SHOT);

        Uri defaultSoundUri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION);
        NotificationCompat.Builder notificationBuilder = new NotificationCompat.Builder(this)
                .setSmallIcon(R.mipmap.ic_launcher)
                .setContentTitle("From eYIC")
                .setContentText(messageBody.get("Title"))
                .setAutoCancel(true)
                .setSound(defaultSoundUri)
                .setContentIntent(pendingIntent);

        NotificationManager notificationManager =
                (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);

        notificationManager.notify(0, notificationBuilder.build());
    }
}