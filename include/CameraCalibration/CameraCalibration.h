#ifndef __CAMERA_CALIBRATION_H__
#define __CAMERA_CALIBRATION_H__
#include <fstream>
#include "CameraCalibration/EdgesSubPix.h"
#include "CameraCalibration/common.h"
#include "CameraCalibration/util.h"
#include "CameraCalibration/refinement.h"
using namespace cv;

namespace xb
{

struct Pattern_Base {};
struct ChessBoard_Pattern : public Pattern_Base {};
struct Circle_Pattern : public Pattern_Base {};
struct Concentric_Pattern : public Pattern_Base {};

class CameraCalibration
{
public:
    CameraCalibration(int points_per_col, int points_per_row, int circle_number, const int polar_number, const std::string& image_path):
                points_per_col_(points_per_col),
                points_per_row_(points_per_row),
                circle_number_(circle_number),
                polar_number_(polar_number),
                image_path_(image_path)
    {
        
    }

    void operator()(Concentric_Pattern)
    {
        cv::Size img_size = ExtractFeatures(Concentric_Pattern());
        cv::Mat cameraMatrix,distCoeffs;
        std::vector<cv::Mat> rvecs,tvecs,rvecs2,tvecs2;
        double reproject_error = cv::calibrateCamera(XYZs_all, uvs_all, img_size, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3);

        //开始保存标定结果
        ofstream os("/home/xubin/Desktop/project/calibration_result/result.txt");
        if (!os)
        {
            // 打开文件失败
            std::cerr << "Error opening file!" << std::endl;
            return;
        }
        os << "重投影误差：" << reproject_error << std::endl;
        os << "内参：" << cameraMatrix << endl;
        os << "畸变参数：" << distCoeffs << endl;
        os.close();
    }

    void operator()(ChessBoard_Pattern)
    {}

private:
void MatchPattern(const cv::Mat& image, std::vector<std::vector<EllipseContour>>& conics, std::vector<std::vector<EllipseContour>>& match_pattern)
    {
        double left_top_cross_ratio = 4.0;
        double right_top_cross_ratio = 2.8;
        double left_bottom_cross_ratio = 2.4;
        double right_bottom_cross_ratio = 1.5;
        Eigen::Vector3f left_top_position, right_top_position, left_bottom_position, right_bottom_position;

        double left_top_error = INT_MAX, right_top_error = INT_MAX, left_bottom_error = INT_MAX, right_bottom_error = INT_MAX;
        // height->长轴，width->短轴
        for(auto& it : conics)
        {
            double cross_ratio = cross_ratio_axis(it.back().retval.size.height, it.front().retval.size.height);
            // std::cout << cross_ratio << std::endl;
            if(abs(cross_ratio - left_top_cross_ratio) < left_top_error)
            {
                left_top_position << it.front().retval.center.x, it.front().retval.center.y, 1;
                left_top_error = abs(cross_ratio - left_top_cross_ratio);
            }
            if(abs(cross_ratio - right_top_cross_ratio) < right_top_error)
            {
                right_top_position << it.front().retval.center.x, it.front().retval.center.y, 1;
                right_top_error = abs(cross_ratio - right_top_cross_ratio);
            }
            if(abs(cross_ratio - left_bottom_cross_ratio) < left_bottom_error)
            {
                left_bottom_position << it.front().retval.center.x, it.front().retval.center.y, 1;
                left_bottom_error = abs(cross_ratio - left_bottom_cross_ratio);
            }
            if(abs(cross_ratio - right_bottom_cross_ratio) < right_bottom_error)
            {
                right_bottom_position << it.front().retval.center.x, it.front().retval.center.y, 1;
                right_bottom_error = abs(cross_ratio - right_bottom_cross_ratio);
            }
        }
        // std::cout << left_top_position << std::endl
        // << left_bottom_position << std::endl 
        // << right_top_position << std::endl
        // << right_bottom_position << std::endl;
        Eigen::Vector3f left_line = left_top_position.cross(left_bottom_position);
        Eigen::Vector3f right_line = right_top_position.cross(right_bottom_position);

        cv::circle(image, Point((int)left_top_position[0], (int)left_top_position[1]), 4, cv::Scalar(0, 0, 255), -1);
        cv::circle(image, Point((int)right_top_position[0], (int)right_top_position[1]), 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(image, Point((int)left_bottom_position[0], (int)left_bottom_position[1]), 4, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, Point((int)right_bottom_position[0], (int)right_bottom_position[1]), 4, cv::Scalar(255, 0, 255), -1);


        std::vector<Eigen::Vector3f> left_row_point, right_row_point;
        for(auto& it : conics)
        {
            if(isOnLine(left_line, it.front().retval.center))
            {
                Eigen::Vector3f left_position(it.front().retval.center.x, it.front().retval.center.y, 1);
                left_row_point.push_back(left_position);
            }
            else if(isOnLine(right_line, it.front().retval.center))
            {
                Eigen::Vector3f right_position(it.front().retval.center.x, it.front().retval.center.y, 1);
                right_row_point.push_back(right_position);
            }
        }
        if(left_row_point.size() != right_row_point.size())
        {
            std::cout << left_row_point.size() << "-----" << right_row_point.size() << std::endl;
            cerr << "detect center error!" << std::endl;
        }
        sort(left_row_point.begin(), left_row_point.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.hnormalized()[1] < b.hnormalized()[1]; });
        sort(right_row_point.begin(), right_row_point.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.hnormalized()[1] < b.hnormalized()[1]; });
        int s[9] = {0, 0, 255, 0, 255, 0, 255, 0, 0};

        for(size_t i = 0; i < left_row_point.size(); i++)
        {
            std::vector<std::vector<EllipseContour>> row_circle;
            Eigen::Vector3f line = left_row_point[i].cross(right_row_point[i]);
            // if(i != 0)
            // {
            //     // 绘制直线
            //     cv::line(image, Point((int)(left_row_point[i][0] / left_row_point[i][2]), (int)(left_row_point[i][1] / left_row_point[i][2])),
            //     Point((int)(right_row_point[i - 1][0] / right_row_point[i - 1][2]), (int)(right_row_point[i - 1][1] / right_row_point[i - 1][2])), cv::Scalar(s[(3 * i) % 9], s[(3 * i + 1) % 9], s[(3 * i + 2) % 9]), 6);
            // }
            // cv::line(image, Point((int)(left_row_point[i][0] / left_row_point[i][2]), (int)(left_row_point[i][1] / left_row_point[i][2])),
            //     Point((int)(right_row_point[i][0] / right_row_point[i][2]), (int)(right_row_point[i][1] / right_row_point[i][2])), cv::Scalar(s[(3 * i) % 9], s[(3 * i + 1) % 9], s[(3 * i + 2) % 9]), 6);
            for(auto& it : conics)
            {
                if(isOnLine(line, it.front().retval.center))  row_circle.push_back(it); 
            }
            // std::cout << row_circle.size() << std::endl;
            sort(row_circle.begin(), row_circle.end(), [](const std::vector<EllipseContour>& a, const std::vector<EllipseContour>& b) { return a.front().retval.center.x < b.front().retval.center.x; });
            for(auto& it : row_circle)
            {
                cv::circle(image, Point((int)it.front().retval.center.x, (int)it.front().retval.center.y), 4, cv::Scalar(s[(3 * i) % 9], s[(3 * i + 1) % 9], s[(3 * i + 2) % 9]), -1);
                match_pattern.push_back(it);
            }
            
        }
    }
    
    void DetectCenter(const cv::Mat& image, std::vector<std::vector<EllipseContour>>& match_pattern, std::vector<Point2f>& image_points)
    {
        for(auto& concentric : match_pattern)
        {
            std::vector<Eigen::Vector3f> polars;
            for(auto& pole : concentric[0].poles)
            {
                polars.push_back(getCoincMatrix(concentric[5].retval) * pole);
            }

            std::vector<cv::Point2f> line_points;
            std::vector<Eigen::Vector3f> lines;
            cv::Vec4f center_line;
            for(auto& polar : polars)
            {
                for(size_t i = 0; i < concentric.size(); i++)
                {
                    auto p = (getCoincMatrix(concentric[i].retval).inverse() * polar);
                    line_points.push_back(std::move(cv::Point2f(p[0] / p[2], p[1] / p[2])));
                    cv::circle(image, Point(p[0] / p[2], p[1] / p[2]), 2, cv::Scalar(0, 0, 255), -1);
                }
                cv::fitLine(line_points, center_line, cv::DIST_L2, 0, 0.01, 0.01);
                lines.push_back(std::move(getLineVec(center_line)));
            }
            std::vector<double> ans = LM_refinement(lines);
            image_points.push_back(Point2f(ans[0], ans[1]));
            // cv::circle(image, Point((int)ans[0], (int)ans[1]), 4, cv::Scalar(0, 0, 255), -1);
        }
    }

    void FilterCircle(std::vector<EllipseContour>& conics, std::vector<std::vector<EllipseContour>>& pattern_set)
    {
        double error = 50;
        bool flag = false;
        std::vector<std::vector<EllipseContour>> pattern;
        for(auto& it : conics)
        {
            if(pattern.size() == 0)
            {
                std::vector<EllipseContour> concentric;
                concentric.push_back(it);
                pattern.push_back(concentric);
            }
            else
            {
                flag = true;
                for(auto& conic : pattern)
                {
                    if(P2P_dist(it.retval.center, conic[0].retval.center) < error)
                    {
                        conic.push_back(it);
                        flag = false;
                    }
                }
                if(flag)
                {
                    std::vector<EllipseContour> concentric;
                    concentric.push_back(it);
                    pattern.push_back(concentric);
                }
            }
        }

        for(auto& it : pattern)
        {
            std::vector<EllipseContour> circle_set;
            sort(it.begin(), it.end(), [](EllipseContour a, EllipseContour b) { return a.retval.size.height < b.retval.size.height; });
            for(size_t i = 0; i < it.size(); i += 2)
            {
                circle_set.push_back(it[i]);
            }
            if(circle_set.size() > circle_number_ - 2)
            {
                pattern_set.push_back(circle_set);
            }
        }
    }

    cv::Size ExtractFeatures(Concentric_Pattern)
    {
        std::vector<String> pictures;
        std::vector<Point3f> object_points;

        //遍历所有的角点
        for (size_t i = 0; i < points_per_col_; i++)
        {
            for (size_t j = 0; j < points_per_row_; j++)
            {
                cv::Point3f singleRealPoint;//一个角点的坐标，初始化三维坐标
                singleRealPoint.x = i;     //10是长/宽，根据黑白格子的长和宽，计算出世界坐标系（x,y,z)
                singleRealPoint.y = j;
                singleRealPoint.z = 0;//假设z=0
                object_points.push_back(singleRealPoint);
            }
        }

        glob(image_path_, pictures);
        
        Mat image, gray_image, filter_image, binary_image, canny_image;
        for(auto it : pictures)
        {
            std::vector<EllipseContour> conics;
            std::vector<Point2f> image_points;
            std::vector<Contour> contours;
            
            image = imread(it, IMREAD_COLOR);
            cvtColor(image, gray_image, COLOR_BGR2GRAY);
            GaussianBlur(gray_image, filter_image, Size(5, 5), 0);
            threshold(filter_image, binary_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            EdgesSubPix(binary_image, 1.0, 20, 40, contours);
            for(const auto& it : contours)
            {
                if(it.points.size() < 5) continue;
                EllipseContour ret(fitEllipse(it.points));
                int step = (int)(it.points.size() / polar_number_ + 1);
                for(size_t i = 0; i < it.points.size(); i += step)
                {
                    ret.poles.push_back(std::move(Eigen::Vector3f(it.points[i].x, it.points[i].y, 1)));
                }
                // ellipse(image, ret, Scalar(0, 0, 255), 4);
                conics.push_back(ret);
            }

            // std::vector<std::vector<Point>> coin;
            // Canny(binary_image, canny_image, 120, 255);
            // cv::findContours(canny_image, coin, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            // for(const auto& it : coin)
            // {
            //     if(it.size() < 5) continue;
            //     EllipseContour ret(fitEllipse(it));
            //     int step = (int)(it.size() / polar_number + 1);
            //     for(size_t i = 0; i < it.size(); i += step)
            //     {
            //         ret.poles.push_back(std::move(Eigen::Vector3f(it[i].x, it[i].y, 1)));
            //     }
            //     // ellipse(image, ret, Scalar(0, 0, 255), 4);
            //     conics.push_back(ret);
            // }

            std::vector<std::vector<EllipseContour>> pattern_set;
            FilterCircle(conics, pattern_set);

            std::vector<std::vector<EllipseContour>> match_pattern;
            MatchPattern(image, pattern_set, match_pattern);
            DetectCenter(image, match_pattern, image_points);
            uvs_all.push_back(image_points);
            XYZs_all.push_back(object_points);

            // cv::imwrite("/home/xubin/Desktop/project/result/01.jpg", image);

            // imshow("image", image);
            // cv::waitKey(10000);
        }
        return image.size();
    }

private:
    const int points_per_col_, points_per_row_;
    const int circle_number_, polar_number_;
    const std::string image_path_;

    std::vector<std::vector<Point2f>> uvs_all;
    std::vector<std::vector<Point3f>> XYZs_all;
};

} // namespace xb

#endif
