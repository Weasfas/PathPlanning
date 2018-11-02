package chooserUI;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileNameExtensionFilter;

/**
 * @author David Yagüe Cuevas
 * @version 1.3
 * @Description Chooser UI component for Astar class
 * @Input arg1 - Layout path: /home/john_doe/... , arg2 - Heuristic to use: Manhattan, Steven or Diagonal (only in diagonal planner), arg3 - Type of planer: H or D, 
 * @since 29/10/2018
 */

public class ChooserUI {
	public String path = "";
	public String mode = "";
	public String heuristic = "";
	
	public ChooserUI(){}
	
	public void runChooser(){
		try{
			JFrame frame = new JFrame();

			String sP = "";

			Object[] PList = {"A* - 4G", "A* - 6G"};
			sP = (String)JOptionPane.showInputDialog(
					frame,
					"What kinf of planner do you want?:\n",
					"Select the planner...",
					JOptionPane.PLAIN_MESSAGE,
					null,
					PList,
					"A* - 4G");

			if(sP == null){
				System.exit(0);
			}

			Object[] HDlist = {"Manhattan", "Steven", "Diagonal"};
			Object[] Hlist = {"Manhattan", "Steven"};

			String sH = "";
			
			if(sP.equals("A* - 4G")){
				sH = (String)JOptionPane.showInputDialog(
						frame,
						"Choose your heuristic:\n",
						"Select the heuristic...",
						JOptionPane.PLAIN_MESSAGE,
						null,
						Hlist,
						"Manhattan");
				mode = "H";
				heuristic = sH;
			}else if(sP.equals("A* - 6G")){
				sH = (String)JOptionPane.showInputDialog(
						frame,
						"Choose your heuristic:\n",
						"Select the heuristic...",
						JOptionPane.PLAIN_MESSAGE,
						null,
						HDlist,
						"Manhattan");
				mode = "D";
				heuristic = sH;
			}else{
				System.exit(0);
			}

	    	JFileChooser chooser = new JFileChooser();
	        FileNameExtensionFilter filter = new FileNameExtensionFilter(
	                "Map layout files", "lay");
	        chooser.setFileFilter(filter);
	        chooser.setDialogTitle("Choose a map layout");
	        int returnVal = chooser.showOpenDialog(null);
	        if(returnVal == JFileChooser.APPROVE_OPTION) {
	        	path = chooser.getSelectedFile().getAbsolutePath();
	        }
	      
        
		}catch (Exception e){
			System.out.println("Something went wrong... " + e.getCause());
		}	
	}
	
}
