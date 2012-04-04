package kinect.kinect;

import java.io.*;
import java.util.*;

import april.util.*;

public class PtsFileConverter
{
    public PtsFileConverter(GetOpt opts)
    {
        FileInputStream fin;
        FileOutputStream fout;
        DataInputStream ins = null;
        DataOutputStream outs = null;
        try {
           fin = new FileInputStream(opts.getString("infile"));
           fout = new FileOutputStream(opts.getString("outfile"));
           ins = new DataInputStream(fin);
           outs = new DataOutputStream(fout);
        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }

        if (ins != null && outs != null) {
            convertFile(ins, outs);
        }
    }

    private void convertFile(DataInputStream ins, DataOutputStream outs)
    {
        BinaryStructureReader bsr = new BinaryStructureReader(ins);
        BinaryStructureWriter bsw = new BinaryStructureWriter(outs);

        try {
            while (true) {
                // Read in data
                int dc = bsr.readInt();
                bsr.blockBegin();
                ArrayList<String> labels = new ArrayList<String>();
                try {
                    while (true) {
                        String label = bsr.readString();
                        System.out.println(label);
                        if (label == null)
                            break;
                        labels.add(label);
                    }
                } catch (Exception ex) {

                }
                System.out.printf("Found %d labels\n", labels.size());
                int nlabels = labels.size();

                // read back junk bytes
                dc = bsr.readInt();

                int npoints = bsr.readInt();
                System.out.println(npoints);
                ArrayList<double[]> points = new ArrayList<double[]>();
                bsr.blockBegin();
                for (int i = 0; i < npoints; i++) {
                    points.add(bsr.readDoubles());
                }
                bsr.blockEnd();

                // Write back to file
                bsw.writeInt(nlabels);
                bsw.blockBegin();
                for (int i = 0; i < nlabels; i++) {
                    bsw.writeString(labels.get(i));
                }
                bsw.blockEnd();

                bsw.writeInt(npoints);
                bsw.blockBegin();
                for (int i = 0; i < npoints; i++) {
                    bsw.writeDoubles(points.get(i));
                }
                bsw.blockEnd();
            }
        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('i', "infile", null, "Input .pts file");
        opts.addString('o', "outfile", null, "Output file for converted file");

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
