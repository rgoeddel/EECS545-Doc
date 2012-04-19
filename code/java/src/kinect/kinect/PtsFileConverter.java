package kinect.kinect;

import java.io.*;
import java.util.*;

import april.util.*;

/** Take in a pts file and output a feature vector file */
public class PtsFileConverter
{
    public PtsFileConverter(GetOpt opts)
    {
        FileInputStream fin;
        DataInputStream ins = null;
        File fout = null;
        try {
           fin = new FileInputStream(opts.getString("infile"));
           ins = new DataInputStream(fin);
           fout = new File(opts.getString("outfile"));
        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }

        if (ins != null && fout != null) {
            convertFile(ins, fout);
        }
    }

    private void convertFile(DataInputStream ins, File fout)
    {
        BinaryStructureReader bsr = new BinaryStructureReader(ins);
        PrintWriter pwout;
        try {
            pwout = new PrintWriter(fout);
        } catch (IOException ioex) {
            ioex.printStackTrace();
            return;
        }

        try {
            while (true) {
                // Read in data
                int nlabels = bsr.readInt();
                bsr.blockBegin();
                ArrayList<String> labels = new ArrayList<String>();
                for (int i = 0; i < nlabels; i++) {
                    String label = bsr.readString();
                    labels.add(label);
                }
                bsr.blockEnd();

                int npoints = bsr.readInt();
                ArrayList<double[]> points = new ArrayList<double[]>();
                bsr.blockBegin();
                for (int i = 0; i < npoints; i++) {
                    points.add(bsr.readDoubles());
                }
                bsr.blockEnd();

                // Write back out to FV file
                pwout.printf("[");
                ArrayList<Double> features = FeatureVec.getFeatureVec(points);
                for (int i = 0; i < features.size(); i++) {
                    pwout.printf("%f ", features.get(i));
                }
                pwout.printf("] {");
                for (int i = 0; i <  labels.size(); i++) {
                    pwout.printf("%s;", labels.get(i));
                }
                pwout.printf("}\n");
                pwout.flush();
            }
        } catch (Exception ex) {
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('i', "infile", null, "Input .pts file");
        opts.addString('o', "outfile", null, "Output feature vector file");

        if (!opts.parse(args)) {
            System.err.println("ERR: "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        PtsFileConverter pfc = new PtsFileConverter(opts);
    }
}
