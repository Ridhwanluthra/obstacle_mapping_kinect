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
 * Created by pankaj on 15/7/16.
 */
public class MyFirebaseMessagingService extends FirebaseMessagingService {
    private static final String TAG = "MyFirebaseMsgService";

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