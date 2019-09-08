
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    static unsigned char img_cnt = 1;
    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;


            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    //float lineSpacing = 0.1; // gap between distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
     /*   putText(topviewImg, to_string(i), cv::Point2f(1400, y), cv::FONT_ITALIC, 0.3, cv::Scalar(255, 0, 0));*/
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
    cv::imwrite("../output/topview/"+to_string(img_cnt)+".png",topviewImg);
    img_cnt++;
    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}



/* clusterKptMatchesWithROI associates a given bounding box with the keypoints it contains.
 * Following steps were followed to realize this.
 * 1) From kptMatches buffer , identify all the points that fall in the given box.
 * 	  Store such points in 'bBkptMatches'
 * 2) Calculate the mean and std of distance of the points in 'bBkptMatches' buffer.
 * 3) Add all the points in bBkptMatches that are with in +/- 2* std of mean to boundingBox.kptMatches.
 *    This removes outliers around 95% of the points would be left after this steps.
 */
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
	cv::Rect bbox;
	cv::Point qPoint,rPoint;
	// Buffer to store the intermediate points
	std::vector<cv::DMatch> bBkptMatches;
	/*std::vector<float>bBkptdistance;*/

	/*Variables to hold mean and std of euclidean distances*/
	double meandistance =0.0;
	double stddistance =0.0;

	/*Load the box coordinates*/
	bbox.x = boundingBox.roi.x;
	bbox.y = boundingBox.roi.y;
	bbox.width = boundingBox.roi.width;
	bbox.height = boundingBox.roi.height;

	/*Loop through each kptMatches , identify points that are in given box*/
	for(auto it = kptMatches.begin();it!=kptMatches.end();it++)
	{
		 qPoint = kptsPrev[it->queryIdx].pt;
		 rPoint = kptsCurr[it->trainIdx].pt;
		 //Check if the current point is in the given box
		 if(bbox.contains(rPoint))
		 {
			 //Store all the points that fall in the given box
			 bBkptMatches.push_back(*it);
			 //Calculate the sum from identified points eculidean distance
			 meandistance+=it->distance;
			 /*bBkptdistance.push_back(it->distance);*/
		 }
	}

	if(bBkptMatches.size()>0)
	{
		//Calculate mean from identified points eculidean distance
		meandistance /=bBkptMatches.size();
		//Calculate std of the selected points.
		for(auto it = bBkptMatches.begin();it!=bBkptMatches.end();it++)
		{
			stddistance +=(it->distance-meandistance)*(it->distance-meandistance);
		}
		stddistance /=bBkptMatches.size();
		stddistance = sqrt(stddistance);

		//Loop through bBkptMatches , store only those points that are +/-2*std of mean.
		for(auto it = bBkptMatches.begin();it!=bBkptMatches.end();it++)
		{
			//Check if the point is in mean+/-2*std , store in final buffer if yes
			if((it->distance>(meandistance-2*stddistance))||
					(it->distance<(meandistance+2*stddistance)))
			{
				//Store the points that are in mean+/-2*std
				boundingBox.kptMatches.push_back(*it);
			}
		}
		//std::cout<<"Success lusterKptMatchesWithROI matching size: "<<boundingBox.kptMatches.size() <<"BoxID: "<< boundingBox.boxID<<std::endl;
	}
	/*else
	{
		std::cout<<"Error lusterKptMatchesWithROI 0 matches BoxID: "<< boundingBox.boxID<<std::endl;
	}*/
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
	// compute distance ratios between all matched keypoints
	vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
	for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
	{ // outer kpt. loop

		// get current keypoint and its matched partner in the prev. frame
		cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
		cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

		for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
		{ // inner kpt.-loop

			double minDist = 100.0; // min. required distance

			// get next keypoint and its matched partner in the prev. frame
			cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
			cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

			// compute distances and distance ratios
			double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
			double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

			if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
			{ // avoid division by zero

				double distRatio = distCurr / distPrev;
				distRatios.push_back(distRatio);
			}
		} // eof inner loop over all matched kpts
	}     // eof outer loop over all matched kpts

	// only continue if list of distance ratios is not empty
	if (distRatios.size() == 0)
	{
		TTC = NAN;
		return;
	}


	// STUDENT TASK (replacement for meanDistRatio)
	std::sort(distRatios.begin(), distRatios.end());
	long medIndex = floor(distRatios.size() / 2.0);
	double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

	double dT = 1 / frameRate;
	TTC = -dT / (1 - medDistRatio);
	// EOF STUDENT TASK
}

/*sortcol is a callback function for sorting the Lidarpoints based on x coordinate
 * used in computeTTCLidar function, to sort prev and curr frame lidar points based
 * on x coordinates in ascending order
 * */
bool sortcol( const LidarPoint & v1,
               const LidarPoint & v2 ) {
 return v1.x < v2.x;
}

/*Function computeTTCLidar computes the TTC based on the LIDAR Data points.
 * Following steps were implemented for calculating TTC
 * 1) Sort previous and current lidar points in ascending order based on x coordinates.
 * 2) Loop through each of the sorted prev lidar points from beginning, find the point with smallest x that has atleast
 *    neigh_thold (100) points within 0.1 meters. Stop searching once such a point is found.
 * 3) Loop through each of the sorted current lidar points from beginning, find the point with smallest x that has atleast
 *    neigh_thold (100) points within 0.1 meters. Stop searching once such a point is found.
 * 4) Use the points identified in step 2 and 3 to calculate TTC
 */
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

	double dT = 1.0f/frameRate;
	// minPrev and minCurr will have closest points from previous and current frames.
	LidarPoint minPrev = {1e9,1e9,1e9,0};
	LidarPoint minCurr = {1e9,1e9,1e9,0};
	// Buffers to hold lidar points in ascending x-axis order
	std::vector<LidarPoint> sorted_XPrev,sorted_XCurr;
	// Neighbourhood count and threshold for choosing points
	unsigned int neigh_count =0, neigh_thold =100;
	// Copy the lidar points
	sorted_XPrev= lidarPointsPrev;
	sorted_XCurr= lidarPointsCurr;
	// Sort the lidar points in ascending order
	std::sort(sorted_XPrev.begin(),sorted_XPrev.end(),sortcol); // Sort in ascending order
	std::sort(sorted_XCurr.begin(),sorted_XCurr.end(),sortcol); // Sort in ascending order

	//Loop through each of the sorted prev lidar points from beginning
	for(auto it = sorted_XPrev.begin();it!=sorted_XPrev.end();it++)
	{
		neigh_count =0;
		//Copy the point into minPrev
		minPrev.x = it->x;
		minPrev.y = it->y;
		minPrev.z = it->z;
		minPrev.r = it->r;
		//Search the entire buffer for closet points
		for(auto it1 = sorted_XPrev.begin();it1!=sorted_XPrev.end();it1++)
			{
				/*//Calculate distance
				float dis = sqrt((minPrev.x-it1->x)*(minPrev.x-it1->x)+
									(minPrev.y-it1->y)*(minPrev.y-it1->y)+
										(minPrev.z-it1->z)*(minPrev.z-it1->z));*/

				//Check if the distance in x direction b/w prev and curr frames is less 0.1 meters
				if(abs(minPrev.x-it1->x)<0.1)
				{
					// Increment the count if dis is less than 0.1mts
					neigh_count++;
				}

			}
		// Exit the loop if the current point has more than 100 neighbours with in 0.1 mts.
		if(neigh_count>=neigh_thold)
			break;

	}

	//std::cout<<"Prev Neigh Count: "<<neigh_count<<std::endl;

	for(auto it = sorted_XCurr.begin();it!=sorted_XCurr.end();it++)
	{
		neigh_count =0;
		//Copy the point into minPrev
		minCurr.x = it->x;
		minCurr.y = it->y;
		minCurr.z = it->z;
		minCurr.r = it->r;

		//Search the entire buffer for closet points
		for(auto it1 = sorted_XCurr.begin();it1!=sorted_XCurr.end();it1++)
			{
			/*//Calculate distance
			float dis = sqrt((minCurr.x-it1->x)*(minCurr.x-it1->x)+
								(minCurr.y-it1->y)*(minCurr.y-it1->y)+
									(minCurr.z-it1->z)*(minCurr.z-it1->z));*/
			//Check if the distance in x direction b/w prev and curr frames is less 0.1 meters
			if(abs(minCurr.x-it1->x)<0.1)
				{
				// Increment the count if dis is less than 0.1mts
					neigh_count++;
				}

			}
		// Exit the loop if the current point has more than 100 neighbours with in 0.1 mts.
		if(neigh_count>=neigh_thold)
			break;
	}
	//std::cout<<"Curr Neigh Count: "<<neigh_count<<std::endl;

	//std::cout<<"Prev Point: "<<minPrev.x<<"Curr Point: "<<minCurr.x<<std::endl;
	// Calculate TTC using identified points
	TTC = minCurr.x * dT / (minPrev.x-minCurr.x);
/*	minCurrPnt = minCurr;
	minPrevPnt = minPrev;*/

}
/* matchBoundingBoxes function implements a mechanism to identify unique box matches b/w prev
 * and curr frame using matched keypoints. This is achieved by following the below steps.
 * 1) Loop through each of the matched keypoints, search in prev and curr frame for the boxes that contain these match points
 * 2) For each matched keypoint match entry is done only if only box in prev and one box in curr frame was identified else the point is discarded
 * 3) No of occurrences of unique prev box id , curr box id pairs is counted.
 * 4) For a give prev box id select the pair with maximum occurrence.
 * */

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
	float shrinkfactor =0.10;
	multimap <int, int> boxMatches;
	/*Loop through each of the match points in matches buffer*/
   for(auto it = matches.begin();it!=matches.end();it++)
   {
	   cv::Point qPoint,rPoint;
	   vector<int> prev_BoxID,curr_BoxID;

	   /*Extract one match point (matched prev and curr frame points) at a time*/
	   qPoint = prevFrame.keypoints[it->queryIdx].pt;
	   rPoint = currFrame.keypoints[it->trainIdx].pt;

	   /*Search the previous frame for the box that contains the match point*/
	   for(auto it1 = prevFrame.boundingBoxes.begin();it1 !=prevFrame.boundingBoxes.end();it1++)
	   {
		   cv::Rect smallerbox;
		   /*Shrink the size of the box to avoid false alarms and
		    * calculate the Right top point , width height of the box*/
		   smallerbox.x = it1->roi.x +it1->roi.width*shrinkfactor/2;
		   smallerbox.y = it1->roi.y +it1->roi.height*shrinkfactor/2;
		   smallerbox.width = it1->roi.width*(1-shrinkfactor);
		   smallerbox.height = it1->roi.height*(1-shrinkfactor);

		   /*Check if the match point is with in the box
		    * if it is then add box id to the buffer*/
		   if(smallerbox.contains(qPoint))
		   {
			   prev_BoxID.push_back(it1->boxID);
		   }

	   }
	   /*Search current frame only if exactly one box ,that contains
	    * the prev match point,was found in the prev frame else skip the match point*/
	   if(prev_BoxID.size()==1)
	   {
		   for(auto it2 = currFrame.boundingBoxes.begin();it2 !=currFrame.boundingBoxes.end();it2++)
		   	   {
		   		   cv::Rect smallerbox1;
		   		   /*Shrink the size of the box to avoid false alarms and
		   			*calculate the Right top point , width height of the box*/
		   		   smallerbox1.x = it2->roi.x +it2->roi.width*shrinkfactor/2;
		   		   smallerbox1.y = it2->roi.y +it2->roi.height*shrinkfactor/2;
		   		   smallerbox1.width = it2->roi.width*(1-shrinkfactor);
		   		   smallerbox1.height = it2->roi.height*(1-shrinkfactor);
		   		   /*Check if the match point is with in the box
		   		    * if it is then add the box id to the buffer*/
		   		   if(smallerbox1.contains(rPoint))
		   		   {
		   			   curr_BoxID.push_back(it2->boxID);
		   		   }
		   	   }
		   /*if exactly one box ,that contains the curr match point,
		    * was found in the curr frame add identified curr , prev box ids
		    * to the intermediate match buffer "boxMatch"else skip the match point*/
		   if(curr_BoxID.size()==1)
		   {
			   boxMatches.insert(pair <int,int>(prev_BoxID[0],curr_BoxID[0]));
		   }
	   }
   }

   /*loop through each of the identified matched boxes, remove duplicate entries
    *i.e a give prev box id can only have one curr box id and there should be only
    * one entry for the pair. Following steps were implemented to realize this
    * 1) Loop through each of the matched boxes (boxMatches)
    * 2) Take first entry of each matched box as key , search boxMatches for all the entries with
    *    this key and store in "result"
    * 3) Iterator through result and count no of times each of the key, values pair is observed
    * 4) Select the key,value combination that has max count   */
   for(auto it3 = boxMatches.begin();it3!=boxMatches.end();)
   {
	   /*Take first entry of each matched box as key*/
	   int unique_key = it3->first;

	   typedef std::multimap<int, int>::iterator MMAPIterator;
	   // Get all pairs with matching unique_key, and store in "result"
	   std::pair<MMAPIterator, MMAPIterator> result = boxMatches.equal_range(unique_key);
	   // Buffer to hold unique value and no of times the value is observed
	   std::unordered_map<int,int> freq;

		// Iterator through result buffer
		for (MMAPIterator it4 = result.first; it4 != result.second; it4++)
		{
			//Search if freq already has the value
			std::unordered_map<int,int>::iterator it6 = freq.find(it4->second);

				// value already present in the map, incremet the count
				if (it6 != freq.end()) {
					it6->second++;	// increment map's value for key 'c'
				}
				// value not found insert as new member
				else {
					freq.insert(std::make_pair(it4->second, 1));
				}
		}
		// Find the value with maximum count.
		int max_count=0;
		int valuewithmaxcount=-1;
		for(auto it5 = freq.begin();it5!=freq.end();it5++)
		{
			if(valuewithmaxcount==-1)
			{
				valuewithmaxcount = it5->first;
				max_count = it5->second;
			}
			else if(max_count<it5->second)
			{
				max_count = it5->second;
				valuewithmaxcount = it5->first;
			}
		}
		// Store the value with max count and unique_key into the final buffer
		if(valuewithmaxcount!=-1)
		{
			bbBestMatches.insert(pair <int,int>(unique_key,valuewithmaxcount));
		}
		// Loop through boxMatches untill next unique key is not found.
	   do {
	       ++it3;
	     } while (it3 != boxMatches.end() && unique_key == it3->first);
	}

}
