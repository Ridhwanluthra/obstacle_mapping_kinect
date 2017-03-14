package ducic.plumbum.com.eyic2017.fcm;

import android.util.Log;

import com.google.firebase.iid.FirebaseInstanceId;
import com.google.firebase.iid.FirebaseInstanceIdService;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<MyFirebaseInstanceIDService.java>
 * Functions: 		<onTokenRefresh>
 * Global Variables:	<->
 *
 */
public class MyFirebaseInstanceIDService extends FirebaseInstanceIdService {

    /*
    *
    * Function Name: 	<onTokenRefresh>
    * Input: 		<Overridden function>
    * Output: 		<Overridden function>
    * Logic: 		<Call made to this function when Firebase Cloud Messaging token is refreshed>
    * Example Call:		<System makes automatic calls>
    *
    */

    @Override
    public void onTokenRefresh() {

        String refreshedToken = FirebaseInstanceId.getInstance().getToken();

        Log.e("TOKEN", refreshedToken);
    }
}
