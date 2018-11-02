package planExample;

import java.io.PrintWriter;

/**
 * @author David Yagüe Cuevas
 * @version 1.2
 * @Description Test class.
 * @Output 1 Files: .arff with execution statistics.
 * @since 29/10/2018
 */

public class TestBattery {
	
	/*public static void main(String[] args) throws Exception {
		runTests();
	}*/
	
	public static void runTests () throws Exception {

        String paths [] = new String [12];
        String semipath = "/home/opcv/Escritorio/Lay/Layout_";
        
        String hueristicsH [] = {"Manhattan", "Steven"};
        String hueristicsD [] = {"Manhattan", "Steven", "Diagonal"};

        for (int ii = 0; ii < paths.length; ++ii) {
            paths[ii] = semipath + Integer.toString(ii) + ".lay";
        }

        PrintWriter writer = new PrintWriter( "/home/opcv/Escritorio/Lay/Output" + ".arff");
        writer.println("@RELATION planTests");
        writer.println();
        writer.println("@ATTRIBUTE planner  {plannerH, plannerD}");
        writer.println("@ATTRIBUTE heuristic  {Manhattan, Steven, Diagonal}");
        writer.println("@ATTRIBUTE layout {Lay0, Lay1, Lay2, Lay3, Lay4, Lay5, Lay6, Lay7, Lay8, Lay9, Lay10, Lay11}");
        writer.println("@ATTRIBUTE nExpanded  NUMERIC");
        writer.println("@ATTRIBUTE costSoFar  NUMERIC");
        writer.println("@ATTRIBUTE execTime  NUMERIC");
        writer.println("@ATTRIBUTE nGoals  NUMERIC");
        writer.println("@ATTRIBUTE mapSize  NUMERIC");
        writer.println("@ATTRIBUTE pathSize  NUMERIC");
        writer.println();
        writer.println("@DATA");
        
        for (int ii = 0; ii < paths.length; ++ii) {
            
            for (int aa = 0; aa < hueristicsH.length; ++aa) {
                
                AStarPlanner aH = new AStarPlanner(paths[ii],hueristicsH[aa]);
                long start = System.currentTimeMillis();
                aH.doSearch();
                long end = System.currentTimeMillis();
                aH.execTime = end-start;
                //aH.writeToFileWithCorrectFormat(paths[ii]);
                
                writer.println("plannerH, " + hueristicsH[aa] +  ", Lay" + ii  + " , " + Integer.toString(aH.n_Expanded) 
                		+ " , " + Integer.toString(aH.costSoFar) + " , " + Double.toString(aH.execTime) + " , " + Integer.toString(aH.n_goals)
                		+ " , " + Integer.toString(aH.getH()*aH.getW()) + " , " + aH.getPath().size());
                
                aH = null;
                
                //System.out.println("Prueba Horizontal de " + paths[ii] + " con " + hueristicsH[aa]);
            }

            for (int aa = 0; aa<hueristicsD.length; ++aa) {
                
                AStarPlannerDiagonal aD = new AStarPlannerDiagonal(paths[ii],hueristicsD[aa]);
                
                long start = System.currentTimeMillis();
                aD.doSearch();
                long end = System.currentTimeMillis();
                aD.execTime = end-start;
                //aD.writeToFileWithCorrectFormat(paths[ii]);
                
                writer.println("plannerD, "  + hueristicsD[aa] +  ", Lay" + ii  + " , " + Integer.toString(aD.n_Expanded) 
                		+ " , " + Integer.toString(aD.costSoFar) + " , " + Double.toString(aD.execTime) + " , " + Integer.toString(aD.n_goals)
                		+ " , " + Integer.toString(aD.getH()*aD.getW()) + " , " + aD.getPath().size());
                
                aD = null;
                
                //System.out.println("Prueba Diagonal de " + paths[ii] + " con " + hueristicsD[aa]);
            }
        }
        
        writer.close();

    }

}
