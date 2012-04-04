/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package kinect.lcmtypes;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class kinect_status_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public byte rgb[];
    public byte depth[];
    public double dx;
    public double dy;
    public double dz;
 
    public kinect_status_t()
    {
        rgb = new byte[921600];
        depth = new byte[614400];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xad825ecccf25de9dL;
 
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(kinect.lcmtypes.kinect_status_t.class))
            return 0L;
 
        classes.add(kinect.lcmtypes.kinect_status_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        outs.write(this.rgb, 0, 921600);
 
        outs.write(this.depth, 0, 614400);
 
        outs.writeDouble(this.dx); 
 
        outs.writeDouble(this.dy); 
 
        outs.writeDouble(this.dz); 
 
    }
 
    public kinect_status_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public kinect_status_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static kinect.lcmtypes.kinect_status_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        kinect.lcmtypes.kinect_status_t o = new kinect.lcmtypes.kinect_status_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.rgb = new byte[(int) 921600];
        ins.readFully(this.rgb, 0, 921600); 
        this.depth = new byte[(int) 614400];
        ins.readFully(this.depth, 0, 614400); 
        this.dx = ins.readDouble();
 
        this.dy = ins.readDouble();
 
        this.dz = ins.readDouble();
 
    }
 
    public kinect.lcmtypes.kinect_status_t copy()
    {
        kinect.lcmtypes.kinect_status_t outobj = new kinect.lcmtypes.kinect_status_t();
        outobj.utime = this.utime;
 
        outobj.rgb = new byte[(int) 921600];
        System.arraycopy(this.rgb, 0, outobj.rgb, 0, 921600); 
        outobj.depth = new byte[(int) 614400];
        System.arraycopy(this.depth, 0, outobj.depth, 0, 614400); 
        outobj.dx = this.dx;
 
        outobj.dy = this.dy;
 
        outobj.dz = this.dz;
 
        return outobj;
    }
 
}
