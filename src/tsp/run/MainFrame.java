package tsp.run;

import java.awt.BorderLayout;

import javax.swing.JFrame;

public class MainFrame extends JFrame
{
	private static final long serialVersionUID = 1L;
	private static MainFrame instance = null;
	
	private MainFrame ()
	{
		this.setLayout (new BorderLayout ());
        this.setSize(600, 800);
        this.setLocationRelativeTo (null);
        this.setDefaultCloseOperation (JFrame.EXIT_ON_CLOSE);
        this.setVisible (Main.DISPLAY_CHART);
	}
	
	public static MainFrame getInstance ()
	{
        if (MainFrame.instance == null)
        	MainFrame.instance = new MainFrame ();
        return MainFrame.instance;	
	}
}
