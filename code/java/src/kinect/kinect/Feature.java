package kinect.kinect;

public class Feature
{
    String feature;
    double prob;

    public Feature(String f, double p)
    {
        feature = f;
        prob = p;
    }

    public Feature copyFeature()
    {
        return new Feature(feature, prob);
    }

}