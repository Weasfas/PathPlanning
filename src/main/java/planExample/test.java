package planExample;

import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.Mat;
import org.bytedeco.javacpp.opencv_core.Point;
import org.bytedeco.javacpp.opencv_core.Scalar;
import org.bytedeco.javacpp.opencv_highgui;
import org.bytedeco.javacpp.opencv_imgcodecs;
import org.bytedeco.javacpp.opencv_imgproc;

import chooserUI.ChooserUI;
import planExample.AStarPlanner;
import planExample.AStarPlannerDiagonal;
import planExample.AStarPlanner.Cell;
import planExample.AStarPlannerDiagonal.CellD;

/**
 * @author David Yague Cuevas
 * @version 1.1
 * @Description Display implementation of A* Algorithm to solve a Path motion maze with n goals and 1 Source using OpenCv.
 * @Input arg1 - Layout path: /home/john_doe/... , arg2 - Heuristic to use: Manhattan, Steven or Diagonal (only in diagonal planner), arg3 - Type of planer: H or D, 
 * @Output 2 Files: Solved maze with the heuristic required and 1 final file with execution statistics.
 * @since 18/10/2018
 */

public class test {
	
	/**
	 * @Description: Method that calculates the position in pixels of a cell in the map.
	 * @Param: temp: Cell object to extract positions, stepSize: dimension of the cells divisions on the map printed.
	 * @return pix: array containing pix[0] - pixelX and pos[1] - pixelY.
	 */
	static int [] getPix(Cell temp, int stepSize){
		int [] pix = new int [2];

		pix[0] = temp.j*stepSize + (stepSize/2) + stepSize;
		pix[1] = temp.i*stepSize + (stepSize/2) + stepSize;

		return pix;
	}
	
	/**
	 * @Description: Method that calculates the position in pixels of a cellD in the map.
	 * @Param: temp: CellD object to extract positions, stepSize: dimension of the cells divisions on the map printed.
	 * @return pix: array containing pix[0] - pixelX and pos[1] - pixelY.
	 */
	static int [] getPixD(CellD temp, int stepSize){
		int [] pix = new int [2];

		pix[0] = temp.j*stepSize + (stepSize/2) + stepSize;
		pix[1] = temp.i*stepSize + (stepSize/2) + stepSize;

		return pix;
	}
	
	/**
	 * @Description: Method that use OpenCv to print the maze on screen.
	 * @Param: a: Planner (Astar - Horizontal and Vertical) to use.
	 */
	static void print4GOpenCv(AStarPlanner a){
		try{
			//Get List and Data from planner and set OpenCv parameters
			ArrayList<Cell> tr = a.getPath();
			Cell [][] td = a.getGrid();
			ArrayList<Integer> ty = a.getOrder();

			int rows = a.getH();
			int cols = a.getW();
			int siz = rows + cols;
			int step = 50;
			double textSize = 1.0;

			if(siz > 50 && siz <= 100){
				step = 25;
				textSize = 0.5;
			}else if(siz > 100){
				step = 15;
				textSize = 0.25;
			}

			// Generate the grid --> Map
			Mat mat = new Mat((rows)*step+step*2, (cols)*step+step*2, opencv_core.CV_8UC4, new Scalar(255, 255, 255, 0));

			int dist = step;

			int width = (int) mat.size().width();
			int height = (int)  mat.size().height();

			for (int i = 0; i < height; i += dist) {
				if (i!=0){

					opencv_imgproc.line(mat, new Point(step, i),  new Point(width-step, i), new Scalar(0, 0));

				}
			}

			for (int i = 0; i < width; i += dist) {

				if (i!=0){

					opencv_imgproc.line(mat, new Point(i, step),  new Point(i, height-step), new Scalar(0, 0));

				}
			}

			// Print the map static positions: Walls, Waypoints, Goals and Start.
			int stepSize = step;

			int aa = 0;
			int bb = 0;

			for (int j = step; j < height-step; j += stepSize) {
				for (int i = step; i < width-step; i += stepSize) {


					if(td[bb][aa]==null){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(0, 0), -1, opencv_core.LINE_4,0);
					}else if (td[bb][aa].value=='G'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(255, 0, 255, 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "G", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else if (td[bb][aa].value=='S'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(255, 51, 0 , 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "S", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else if(td[bb][aa].value=='W'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(102, 255, 102, 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "W", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else{
						//Nothing to do
					}

					++aa;

				}
				if (aa==cols) {
					aa = 0;
					++bb;
				}
			}

			// Show the calculated path of the planner
			String name = "Map";

			opencv_highgui.namedWindow(name, opencv_highgui.CV_WINDOW_AUTOSIZE);

			// Need to order the reconstruct path: from last to first on each of the goals.
			int index = 0;
			int index2 = 0;

			for(int control = 0; control < ty.size(); ++control){
				index = ty.get(control);
				List<Cell> te = tr.subList(index2,index);

				for(int p = te.size()-1; p >= 0 ; --p){
					Cell temp = te.get(p);
					int [] pos = getPix(temp, stepSize);

					if(p==0){
						opencv_imgproc.circle(mat, new Point(pos[0], pos[1]), step/3, new Scalar(255, 204, 51, 0), -1, opencv_core.LINE_4, 0);
					}else{
						opencv_imgproc.circle(mat, new Point(pos[0], pos[1]), step/3, new Scalar(51, 173, 255, 0), -1, opencv_core.LINE_4, 0);
					}

					
					opencv_highgui.imshow(name, mat);
					opencv_highgui.waitKey(100);

				}

				index2=index;
			}

			// Display map
			opencv_highgui.imshow(name, mat);
			opencv_highgui.waitKey();
			opencv_highgui.destroyAllWindows();

			String pah = test.class.getProtectionDomain().getCodeSource().getLocation().getPath();
			pah = pah + "Map_with_path.bmp";
			
			opencv_imgcodecs.imwrite(pah, mat);
			
			System.out.println("Search image write in: " + pah);

		}catch (IndexOutOfBoundsException r){
			System.err.println("Bad argumentss. Try again." + r.getCause());
		}catch(Exception e){
			System.err.println("Bad argument. Try again. "+ e.getMessage());
		}
	}

	/**
	 * @Description: Method that use OpenCv to print the maze on screen.
	 * @Param: a: Planner (Astar - Horizontal, Vertical and Diagonal) to use.
	 */
	static void print6GOpenCv(AStarPlannerDiagonal a){
		try{
			//Get List and Data from planner and set OpenCv parameters
			ArrayList<CellD> tr = a.getPath();
			CellD [][] td = a.getGrid();
			ArrayList<Integer> ty = a.getOrder();

			int rows = a.getH();
			int cols = a.getW();
			int siz = rows + cols;
			int step = 50;
			double textSize = 1.0;

			if(siz > 50 && siz <= 100){
				step = 25;
				textSize = 0.5;
			}else if(siz > 100){
				step = 15;
				textSize = 0.25;
			}

			// Generate the grid --> Map
			Mat mat = new Mat((rows)*step+step*2, (cols)*step+step*2, opencv_core.CV_8UC4, new Scalar(255, 255, 255, 0));

			int dist = step;

			int width = (int) mat.size().width();
			int height = (int)  mat.size().height();

			for (int i = 0; i < height; i += dist) {
				if (i!=0){

					opencv_imgproc.line(mat, new Point(step, i),  new Point(width-step, i), new Scalar(0, 0));

				}
			}

			for (int i = 0; i < width; i += dist) {

				if (i!=0){

					opencv_imgproc.line(mat, new Point(i, step),  new Point(i, height-step), new Scalar(0, 0));

				}
			}

			
			// Print the map static positions: Walls, Waypoints, Goals and Start.
			int stepSize = step;

			int aa = 0;
			int bb = 0;

			for (int j = step; j < height-step; j += stepSize) {
				for (int i = step; i < width-step; i += stepSize) {


					if(td[bb][aa]==null){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(0, 0), -1, opencv_core.LINE_4,0);
					}else if (td[bb][aa].value=='G'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(255, 0, 255, 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "G", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else if (td[bb][aa].value=='S'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(255, 51, 0 , 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "S", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else if(td[bb][aa].value=='W'){
						opencv_imgproc.rectangle(mat, new Point (i, j), new Point(i+step,j+step), new Scalar(102, 255, 102, 0), -1, opencv_core.LINE_4,0);
						opencv_imgproc.putText(mat, "W", new Point(i + ((int) step/3), j + 5 + ((int) step/2)), opencv_core.FONT_HERSHEY_SCRIPT_COMPLEX , textSize, new Scalar(0, 0));
					}else{
						//Nothing to do
					}

					++aa;

				}
				if (aa==cols) {
					aa = 0;
					++bb;
				}
			}

			// Show the calculated path of the planner
			String name = "Map";

			opencv_highgui.namedWindow(name, opencv_highgui.CV_WINDOW_AUTOSIZE);

			// Need to order the reconstruct path: from last to first on each of the goals.
			int index = 0;
			int index2 = 0;

			for(int control = 0; control < ty.size(); ++control){
				index = ty.get(control);
				List<CellD> te = tr.subList(index2,index);

				for(int p = te.size()-1; p >= 0 ; --p){
					CellD temp = te.get(p);

					int [] pos = getPixD(temp, stepSize);

					if(p==0){
						opencv_imgproc.circle(mat, new Point(pos[0], pos[1]), step/3, new Scalar(255, 204, 51, 0), -1, opencv_core.LINE_4, 0);
					}else{
						opencv_imgproc.circle(mat, new Point(pos[0], pos[1]), step/3, new Scalar(51, 173, 255, 0), -1, opencv_core.LINE_4, 0);
					}

					
					opencv_highgui.imshow(name, mat);
					opencv_highgui.waitKey(100);

				}

				index2=index;
			}

			// Display map
			opencv_highgui.imshow(name, mat);
			opencv_highgui.waitKey();
			opencv_highgui.destroyAllWindows();

			opencv_imgcodecs.imwrite("Map_with_path.bmp", mat);

		}catch (IndexOutOfBoundsException r){
			System.err.println("Bad argumentss. Try again." + r.getCause());
		}catch(Exception e){
			System.err.println("Bad argument. Try again. "+ e.getMessage());
		}
	}
	
	public static void main(String[] args) throws Exception {

		ChooserUI chooser = new ChooserUI();
		chooser.runChooser();
		
		String path = chooser.path;                //args[0];		//Layout Path
		String heurist = chooser.heuristic;        //args[1];	//Heuristic function to use
		String tipo = chooser.mode;                // args[2];		//Type of planner to use

		
		if(tipo.equals("D")){ //AStarDiagonal
			try{
				AStarPlannerDiagonal a = new AStarPlannerDiagonal(path,heurist);
				long start = System.currentTimeMillis();
				a.doSearch();
				long end = System.currentTimeMillis();
				a.execTime = end-start;
				a.writeToFileWithCorrectFormat(path);
				System.out.print("Search finished.\nHeuristic used: " + heurist + ".\nOutput files generated:\n" + 
						path + ".output\n" + path + ".statistics");

				System.out.println();

				print6GOpenCv(a);
				
			}catch (IndexOutOfBoundsException r){
				System.err.println("Bad argument. Try again. " + r.getCause());
			}catch(Exception e){
				System.err.println("Bad argument. Try again. "+ e.getCause());
			}	
		}else if(tipo.equals("H")){ //NormalAstar
			
			try{
				AStarPlanner a = new AStarPlanner(path,heurist);
				long start = System.currentTimeMillis();
				a.doSearch();
				long end = System.currentTimeMillis();
				a.execTime = end-start;
				a.writeToFileWithCorrectFormat(path);
				System.out.print("Search finished.\nHeuristic used: " + heurist + ".\nOutput files generated:\n" + 
						path + ".output\n" + path + ".statistics");

				System.out.println();

				print4GOpenCv(a);
				
			}catch (IndexOutOfBoundsException r){
				System.err.println("Bad argument. Try again. " + r.getCause());
			}catch(Exception e){
				System.err.println("Bad argument. Try again. "+ e.getCause());
			}
			
		}else {
			throw new Exception();
		}
		
		System.exit(0);
		
	}

}
