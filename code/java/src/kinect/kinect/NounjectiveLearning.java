package kinect.kinect;

import april.vis.*;
import april.jmat.*;
import april.util.*;

import lcm.lcm.*;
import kinect.lcmtypes.*;

import kinect.classify.*;
import java.io.*;
import java.nio.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.*;

/* To Do:
 *  - Send out an LCM message at every frame announcing what is being see
 *  - Track objects between frames
 *  - Add features for shape recognition to FeatureVec
 */


public class NounjectiveLearning implements LCMSubscriber
{
    final static int FRAME_WIDTH = 640;
    final static int FRAME_HEIGHT = 480;

    static int initialColorThresh = 13;
    static double initialUnionThresh = 0.5;
    static double  initialRansacThresh = .02;
    static double  initialRansacPercent = .1;

    static DataAggregator da;
    static Segment segment;
    GetOpt opts;
    VisWorld vw, vw2;
    VisLayer vl, vl2;
    static VisWorld.Buffer vb;
    kinect_status_t ks;
    static LCM lcm = LCM.getSingleton();
    Classifier classify = new Classifier();

    public NounjectiveLearning(GetOpt opts_)
    {
        opts = opts_;

        // Initialize all ov Vis stuff.
        vw = new VisWorld();
        vw2 = new VisWorld();
        vl = new VisLayer(vw);
        vl2 = new VisLayer(vw2);
        double pct = 0.25;
        vl2.layerManager = new DefaultLayerManager(vl2, new double[]{0.0, 1.0-pct, pct, pct});
        vl2.cameraManager.fit2D(new double[2], new double[] {kinect_status_t.WIDTH, kinect_status_t.HEIGHT}, true);
        VisCanvas vc = new VisCanvas(vl);
        vc.addLayer(vl2);
        vb = vw.getBuffer("Point_Buffer");

        //Set up initial camera view
        vl.cameraManager.uiLookAt(new double[] {0.0, 0.0, 5.0},// Camera position
                                  new double[] {0.0, 0.0, 0.0},// Point looking at
                                  new double[] {0.0, 1.0, 0.0},// Up
                                  false);

        // Set up adjustable parameters, buttons
        ParameterGUI pg = new ParameterGUI();
        pg.addIntSlider("cThresh", "Color Threshold", 1, 100, initialColorThresh);
        pg.addDoubleSlider("uThresh", "Union Threshold", 0.001, 0.5, initialUnionThresh);
        pg.addDoubleSlider("rThresh", "Ransac Threshold", .001, .1, initialRansacThresh);
        pg.addDoubleSlider("rPercent", "Ransac Percent", .01, .3, initialRansacPercent);

        pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("cThresh")) {
                        da.colorThresh = pg.gi("cThresh");
                    }
                    else if (name.equals("uThresh")) {
                        da.unionThresh = pg.gd("uThresh");
                    }
                    else if (name.equals("rThresh")) {
                        da.ransacThresh = pg.gd("rThresh");
                    }
                    else if (name.equals("rPercent")) {
                        da.ransacPercent = pg.gd("rPercent");
                    }
                }
            });

        // JFrame set up
        JFrame frame = new JFrame("Learning Objects");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        frame.add(vc, BorderLayout.CENTER);
        frame.setSize(2*FRAME_WIDTH, FRAME_HEIGHT);
        vb = vw.getBuffer("object segmenting");
        frame.add(pg, BorderLayout.SOUTH);
        frame.setVisible(true);

        //initialize lcm and start receiving data
        LCM myLCM = LCM.getSingleton();
        myLCM.subscribe("KINECT_STATUS", this);
    }


    /** Upon recieving a message from the Kinect, translate each depth point into
     ** x,y,z space and find the pixel color for it.  Then draw it.
     **/
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            ks = new kinect_status_t(ins);
        }catch (IOException e){
            e.printStackTrace();
            return;
        }

        da.currentPoints = new ArrayList<double[]>();
        da.coloredPoints = new ArrayList<double[]>();

        for(int y=0; y<ks.HEIGHT; y++){
            for(int x=0; x<ks.WIDTH; x++){
                int i = y*ks.WIDTH + x;
                int d = ((ks.depth[2*i+1]&0xff) << 8) |
                        (ks.depth[2*i+0]&0xff);
                double[] pKinect = KUtils.getXYZRGB(x, y, da.depthLookUp[d], ks);
                double[] pWorld = KUtils.getWorldCoordinates(new double[]{
                        pKinect[0], pKinect[1], pKinect[2]});
                da.currentPoints.add(new double[]{pWorld[0], pWorld[1], pWorld[2], pKinect[3]});
                //da.currentPoints.add(pKinect);
            }
        }

        sendMessage();
        draw3DImage();
        draw2DImage();
    }

    public void sendMessage()
    {
        observations_t obs = new observations_t();
        obs.utime = TimeUtil.utime();

        ArrayList<object_data_t> obsList = new ArrayList<object_data_t>();
        ArrayList<String> sensList = new ArrayList<String>();

        Collection c = da.objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo obj = (ObjectInfo)itr.next();
            double[] bb = FeatureVec.boundingBox(obj.points);
            double[] xyzrpy = new double[]{(bb[0]+bb[3])/2.0,
                                           (bb[1]+bb[4])/2.0,
                                           (bb[2]+bb[5])/2.0,
                                           0,0,0};

            // Get features and corresponding classifications for this object
            String input = FeatureVec.featureString(obj.points);
            String adjective = classify.classify(input); // Currenty only one being returned
            categorized_data_t[] data = new categorized_data_t[1];
            categorized_data_t d = new categorized_data_t();
            d.cat = new category_t();
            d.label = new String[]{adjective};
            d.len = 1;
            d.confidence = new double[]{.9};
            data[0] = d;

            // Create object data for lcm
            object_data_t obj_data = new object_data_t();
            obj_data.utime = TimeUtil.utime();
            obj_data.id = obj.repID;
            obj_data.cat_dat = data;
            obj_data.num_cat = obj_data.cat_dat.length;
            obj_data.pos = xyzrpy;
            obj_data.bbox = new double[][]{{bb[0], bb[1], bb[2]},{bb[3], bb[4], bb[5]}};
            obsList.add(obj_data);
        }

        obs.click_id = -1;
        obs.sensables = sensList.toArray(new String[0]);
        obs.nsens = obs.sensables.length;
        obs.observations = obsList.toArray(new object_data_t[0]);
        obs.nobs = obs.observations.length;

        lcm.publish("OBSERVATIONS",obs);
    }


    public void draw3DImage()
    {
        if(da.currentPoints.size() <= 0){ return; }

        segment.unionFind();

        VisColorData cd = new VisColorData();
        VisVertexData vd = new VisVertexData();
        for(double[] p: da.coloredPoints){
            vd.add(new double[]{p[0], p[1], p[2]});
            cd.add((int) p[3]);
        }

        Collection c = da.objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo obj = (ObjectInfo)itr.next();
            VzText text = new VzText(Integer.toString(obj.repID));
            double[] bb = FeatureVec.boundingBox(obj.points);
            double[] xyz = new double[]{(bb[0]+bb[3])/2.0,
                                           (bb[1]+bb[4])/2.0,
                                           (bb[2]+bb[5])/2.0};
            VisChain chain = new VisChain(LinAlg.translate(xyz),LinAlg.scale(.005),text);
            vb.addBack(chain);
        }

        vb.addBack(new VisLighting(false, new VzPoints
                                   (vd, new VzPoints.Style(cd, 1.0))));
        vb.swap();
    }


    public void draw2DImage()
    {
        if(da.currentPoints.size() <= 0){ return; }

        BufferedImage im = new BufferedImage(ks.WIDTH, ks.HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            buf[i] = ks.rgb[i+2];   // B
            buf[i+1] = ks.rgb[i+1]; // G
            buf[i+2] = ks.rgb[i];   // R
        }

        VisWorld.Buffer vbim = vw2.getBuffer("pictureInPicture");
        vbim.addBack(new VzImage(im, VzImage.FLIP));
        vbim.swap();
    }


    public static void main(String args[])
    {
        // Define possible commandline arguments
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");

        if (!opts.parse(args)) {
            System.err.println("ERR: Opts error - " + opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        // Set up data aggregator and segmenter
        da = new DataAggregator(false);
        da.colorThresh = initialColorThresh;
        da.unionThresh = initialUnionThresh;
        da.ransacThresh = initialRansacThresh;
        da.ransacPercent = initialRansacPercent;
        da.depthLookUp = KUtils.createDepthMap();
        boolean colorSegments = true;
        segment = new Segment(da, colorSegments);

        NounjectiveLearning nl = new NounjectiveLearning(opts);
    }
}
