package kinect.ispy;

import java.awt.BorderLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextArea;
import javax.swing.JTextField;

import kinect.classify.KNN;
import kinect.kinect.DataAggregator;
import kinect.kinect.Segment;
import april.vis.VisWorld;

public class TrainingBox extends JFrame{
	final static int WIDTH = 400;
	final static int HEIGHT = 150;
	
	private JTextField textField;
	private JButton finishButton;
	
	public TrainingBox(){
		super("Add Training Labels");
    this.setSize(WIDTH, HEIGHT);
    this.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
    this.setLayout(new BorderLayout());       
    
    JTextArea label = new JTextArea("Type the new label into the box," +
    		"then click objects on the screen that label applies to. " +
    		"Each click will add a training example for that object. " +
    		"Click 'Finish' when you are done'");
    label.setLineWrap(true);
    label.setWrapStyleWord(true);
    this.add(label, BorderLayout.NORTH);
    
    textField = new JTextField();
    this.add(textField, BorderLayout.CENTER);
    
    finishButton = new JButton("Finish");
    this.add(finishButton, BorderLayout.SOUTH);
    finishButton.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				setVisible(false);
			}
    });
        
	}
	
	public String getText(){
		return textField.getText();
	}
	
	public void clearText(){
		textField.setText("");
	}
}
