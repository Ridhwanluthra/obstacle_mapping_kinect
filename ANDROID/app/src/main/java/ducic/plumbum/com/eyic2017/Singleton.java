package ducic.plumbum.com.eyic2017;

import android.support.multidex.MultiDexApplication;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.toolbox.Volley;

/**
 *
 * Project Name: 	<Visual Perception For The Visually Impaired>
 * Author List: 		Pankaj Baranwal
 * Filename: 		<Singleton.java>
 * Functions: 		<onCreate, getInstance, getRequestQueue, addToRequestQueue>
 * Global Variables:	<String TAG, Singleton mInstance, RequestQueue requestQueue>
 *
 */

public class Singleton extends MultiDexApplication {

    public final String TAG = Singleton.class.getSimpleName();
    private static Singleton mInstance;
    private RequestQueue requestQueue;

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
    public void onCreate() {
        super.onCreate();
        mInstance = this;
        requestQueue = getRequestQueue();
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

    public static synchronized Singleton getInstance() {
        return mInstance;
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

    public RequestQueue getRequestQueue() {
        if (requestQueue == null)
            requestQueue = Volley.newRequestQueue(getApplicationContext());

        return requestQueue;
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

    public <T>void addToRequestQueue(Request<T> req) {
        req.setTag(TAG);
        getRequestQueue().add(req);
    }
}
