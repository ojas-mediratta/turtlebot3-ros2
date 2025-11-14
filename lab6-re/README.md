# Simple, Poorly Functioning KNN Classifier. 

It will train and test each time you run the code

example_knn.py - This code runs considering the full image without any processing. The KNN classifier acts only on the raw picture pixel intensity values. The resulting model achieves around 1/3 accuracy.

There are many optional arguments in the code, run python exampleKNN -h for them all. The only required argument is the file path to the directory containing the images and labels.txt file.

Example run:
    python3 example_knn.py -p ./2024F_imgs/ -r 0.6 -k 5