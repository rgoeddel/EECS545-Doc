package kinect.classify;
import java.util.regex.Pattern;
import java.util.regex.Matcher;
import java.io.*;
import de.bwaldvogel.liblinear.*;

public class Classifier
{

    public static String [] colors = new String [6];
    public Classifier()
    {
	colors[0] = "red";
	colors[1] = "blue";
	colors[2] = "yellow";
	colors[3] = "green";
	colors[4] = "purple";
	colors[5] = "orange";
    }

    /** createTestFeatures
     * @param String test -list of features of object
     * @returns FeatureNode [] array of features used to test object
     **/
    private static FeatureNode[] createTestFeatures(String test)
    {
	FeatureNode [] testformatted = new FeatureNode [6];

	String reg = "[0-9]{1,}+(\\.[0-9]{1,})";
	Pattern p = Pattern.compile(reg);

	double [] data = new double [12];
	Matcher m = p.matcher(test);
	int i = 0;

	//get first 12 features- RGB vvv HSV vvv
	while (m.find() && i < 12)
	{
	    data[i] = Double.valueOf(m.group()).doubleValue();
	    i++;
	}
	//output RGB HSV to feature node array
	testformatted[0] = new FeatureNode(1, data[0]);
	testformatted[1] = new FeatureNode(2, data[1]);
	testformatted[2] = new FeatureNode(3, data[2]);
	testformatted[3] = new FeatureNode(4, data[6]);
	testformatted[4] = new FeatureNode(5, data[7]);
	testformatted[5] = new FeatureNode(6, data[8]);

	return testformatted;
    }

    /** classifierConfidence
     * @param String positiveColor- for this test, FeatureNode[] features of
     *        current object testing
     * @returns double -confidence level of given color vs all others
     **/
    public static double classifierConfidence(FeatureNode [] test, String positiveColor)
    {
	//System.out.print(positiveColor + "(");
	int totalcnt = 0;
	int truecnt = 0;
	for (int j = 0; j < 6; j++)
	{
	    //skip when color is the same
	    if (positiveColor.equals(colors[j]))
		continue;

	    try
	    {
		// positive and negative examples for this run
		String pos = positiveColor;
		String neg = colors[j];

		FileInputStream fstream =
		    new FileInputStream("../training/train" + pos + neg + ".TRAIN.model");
		DataInputStream in = new DataInputStream(fstream);
		BufferedReader r = new BufferedReader(
		    new InputStreamReader(in));
		//load trained model for these two colors
		Model m = Linear.loadModel(r);
		//run predictor using model and object features
		int ret = Linear.predict(m, test);
		if (ret == 1)
		    truecnt++;
		totalcnt++;
		in.close();

	    }
	    catch (Exception e)
	    {
		System.err.println("Error: " + e.getMessage());
	    }
	}
	//return percentage of correct classifications with this color
	return ((double)truecnt)/(double)totalcnt*100.0;
    }

    /** classify
     * @param String args - features of object to test
     *
     * @returns String - best color selected by classifiers
     **/
    public static String classify(String args)
    {
	String testFeatures = args;

	double [] conf = new double [6];
        // use test feature string to build test.Test file for liblinear
	FeatureNode [] test = createTestFeatures(testFeatures);

	// run binary svm for each color against each color to determine
	// which color classification is best
	for (int i = 0; i < 6; i++)
	{
	    conf[i] = classifierConfidence(test, colors[i]);
	    //System.out.print(conf[i] + "%) ");
	}

	// find and report best classifier color
	double max = 0.0;
	int maxi = 0;
	for (int k = 0; k < 6; k++)
	{
	    if (conf[k] > max)
	    {
		max = conf[k];
		maxi = k;
	    }
	}

	return colors[maxi];
    }
}
