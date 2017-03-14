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
    * Function Name: 	<Function Name>
    * Input: 		<Inputs (or Parameters) list with description if any>
    * Output: 		<Return value with description if any>
    * Logic: 		<Description of the function performed and the logic used in the function>
    * Example Call:		<Example of how to call this function>
    *
    */

    @Override
    public void onTokenRefresh() {

        String refreshedToken = FirebaseInstanceId.getInstance().getToken();

        Log.e("TOKEN", refreshedToken);
    }
}
