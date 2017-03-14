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
    * Function Name: 	<onCreate>
    * Input: 		<Bundle savedInstanceState(For saving and retrieving data)>
    * Output: 		<void>
    * Logic: 		<Equivalent to 'main' function in JAVA>
    * Example Call:		<Called automatically by OS>
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
    * Function Name: 	<getInstance>
    * Input: 		<void>
    * Output: 		<Singleton (Returns instance of this class)>
    * Logic: 		<Returns instance of this class. Since it is synchronized, only once instance at a time>
    * Example Call:		<getInstance()>
    *
    */

    public static synchronized Singleton getInstance() {
        return mInstance;
    }

    /*
    *
    * Function Name: 	<getRequestQueue>
    * Input: 		<void>
    * Output: 		<RequestQueue(Volley request queue is returned)>
    * Logic: 		<If a queue does not exist, a new queue is sent, else the existing queue is returned>
    * Example Call:		<getRequestQueue()>
    *
    */

    public RequestQueue getRequestQueue() {
        if (requestQueue == null)
            requestQueue = Volley.newRequestQueue(getApplicationContext());

        return requestQueue;
    }

    /*
    *
    * Function Name: 	<addToRequestQueue>
    * Input: 		<Request<T> req(The POST, GET, PUT, DELETE request to be sent)>
    * Output: 		<void>
    * Logic: 		<a new request is added to the exisitng queue>
    * Example Call:		<addToRequestQueue(new Request())>
    *
    */

    public <T>void addToRequestQueue(Request<T> req) {
        req.setTag(TAG);
        getRequestQueue().add(req);
    }
}
