#include <iostream>
#include "sfm.h"
#include "includes.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/xfeatures2d.hpp>

using namespace cv;

int main() {
    SFM sfm(xfeatures2d::SIFT::create(0, 3, 0.04, 10), 
        DescriptorMatcher::create("BruteForce"));
    string imagesFolder("datasets/fountain_dense_images");

    vector<string> imagesDir;
    if (!imagesFolder.empty()) {
        using namespace boost::filesystem;
        path dirpath(imagesFolder);
        if (not exists(dirpath) or not is_directory(dirpath)) {
            cerr<<"cannot open directory"<<imagesFolder<<endl;
            return false;
        }
        for (directory_entry &x : directory_iterator(dirpath)) {
            string extension = x.path().extension().string();
            boost::algorithm::to_lower(extension);
            if (extension == ".jpg" or extension == ".png") {
                imagesDir.push_back(x.path().string());
            }
        }
        if (imagesDir.size() < 0) {
            cerr << "unable to find files in image directory"<<endl;
            return false;
        }
        sort(imagesDir.begin(), imagesDir.end());
    }
    sfm.addImages(imagesDir, Camera::Ptr(new Camera(2759.48, 2764.16, 1520.69, 1006.81)));
    return 0;
}
