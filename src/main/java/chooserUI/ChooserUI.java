package chooserUI;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileNameExtensionFilter;

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
					"Customized Dialog 1",
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
						"Select the planner type",
						JOptionPane.PLAIN_MESSAGE,
						null,
						Hlist,
						"ham");
				mode = "H";
				heuristic = sH;
			}else if(sP.equals("A* - 6G")){
				sH = (String)JOptionPane.showInputDialog(
						frame,
						"Choose your heuristic:\n",
						"Select ",
						JOptionPane.PLAIN_MESSAGE,
						null,
						HDlist,
						"ham");
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
