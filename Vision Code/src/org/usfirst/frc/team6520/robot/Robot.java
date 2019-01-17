package org.usfirst.frc.team6520.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot {

	double centerX = 0, centerY = 0;
	// các biến chỉ tọa độ X, Y trọng tâm của vật thể được detect

	Preferences prefs = Preferences.getInstance();
	// tạo file preference mới cho smartdashboard

	public void vision() {
		
		//khởi tạo camera và đặt resolution
		//resolution khá là quan trọng vì hệ thống điều khiển sân đấu có
		//giới hạn về bandwidth nên cần để dữ liệu thấp
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);

		//khởi tạo CvSink là một vật thể host video từ cam
		CvSink cvSink = CameraServer.getInstance().getVideo();
		//khởi tạo vật thể để output processed feed đến smartdashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("Vision", 320, 240);

		//mat = ma trận
		//tạo mat để thể hiện dữ liệu ảnh
		Mat mat = new Mat();
		//tạo mat thể hiện kernel sử dụng để xử lý (sẽ nói sau).
		//MORPH_RECT tức là hình dạng của kernel này là hình chữ nhật, kích cỡ 3x3
		Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
		
		//tạo danh sách để lưu lại những object detect được
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		
		//một mat nào đó cần thiết cho thuật tìm contour
		Mat hierarchy = new Mat();

		//load các giá trị từ file preference, có default backup nếu pref chưa tồn tại
		int R = prefs.getInt("R", 0); //màu đỏ
		int G = prefs.getInt("G", 0);	//màu lục
		int B = prefs.getInt("B", 0);	//màu xanh lam
		int RE = prefs.getInt("RE", 20);	//khoảng của đỏ
		int GE = prefs.getInt("GE", 20);	//khoảng của lục
		int BE = prefs.getInt("BE", 20);	//khoảng của xanh

		//đưa các giá trị đó lên smartdashboard
		SmartDashboard.putNumber("R", R);
		SmartDashboard.putNumber("G", G);
		SmartDashboard.putNumber("B", B);
		SmartDashboard.putNumber("RE", RE);
		SmartDashboard.putNumber("GE", GE);
		SmartDashboard.putNumber("BE", BE);

		while (true) {
			
			//lưu các giá trị vào preference để khi bot reset hoặc disable/reenable
			//thì không mất giá trị
			prefs.putInt("R", R);
			prefs.putInt("G", G);
			prefs.putInt("B", B);
			prefs.putInt("RE", RE);
			prefs.putInt("GE", GE);
			prefs.putInt("BE", BE);
			
			
			if (cvSink.grabFrame(mat) == 0) {
				//nếu cvSink không vớ được ảnh thì skip loop
				//failsafe measure
				outputStream.notifyError(cvSink.getError());
				continue;
			}

			//lấy giá trị từ dashboard đưa vào robot
			R = (int) SmartDashboard.getNumber("R", 255);
			G = (int) SmartDashboard.getNumber("G", 255);
			B = (int) SmartDashboard.getNumber("B", 255);
			RE = (int) SmartDashboard.getNumber("RE", 20);
			GE = (int) SmartDashboard.getNumber("GE", 20);
			BE = (int) SmartDashboard.getNumber("BE", 20);

			//convert ảnh trong mat từ định dạng RGB ra HSV
			Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
			
			//thuật toán để loại các pixel không nằm trong khoảng cho trước
			//và giữ lại những pixel nằm trong khoảng cho trước
			//khoảng của mỗi màu là (màu - lỗi màu < pixel < màu + lỗi màu)
			Core.inRange(mat, new Scalar(R - RE, G - GE, B - BE), new Scalar(R + RE, G + GE, B + BE), mat);
			
			//sử dụng kernel để loại bỏ các lỗ hổng trong các cụm pixel còn lại
			Imgproc.dilate(mat, mat, kernel);

			//thuật toán tìm viền ngoài của các cụm pixel
			//từ đó group các cụm pixel lại thành vật thể
			//danh sách contour được đưa vào list đã tạo
			Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

			double Xsum = 0;
			int count = 0;
			
			//xử lý từng contour tìm được
			for (int i = 0; i < contours.size(); i++) {
				
				//tạo một bounding box hình chữ nhật bao quanh contour
				Rect boundRect = Imgproc.boundingRect(contours.get(i));
				
				//nếu kích cỡ của contour lớn hơn 1 giá trị
				//dùng để loại các vật thể nhiễu có thể lẫn vào
				if (Imgproc.contourArea(contours.get(i)) > 500) {
					
					//tính tọa độ trọng tâm của bounding box
					centerX = boundRect.x + (boundRect.width / 2);
					centerY = boundRect.y + (boundRect.height / 2);
					
				}
			}
			//đưa các giá trị ra dashboard
			SmartDashboard.putString("Coordinates", centerX + ", " + centerY);

			//đưa output ra dashboard để tiện theo dõi
			outputStream.putFrame(mat);
		}

		//Màu của Vision Target theo tính toán của Đỗ Hoàng Minh:
		//R: 70, RE: 20
		//G: 35, GE: 10
		//B: 245, BE: 10
	}
}