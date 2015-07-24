import argparse
from util import readData
from kalman_filter import predict

def main():
    parser = argparse.ArgumentParser(description='Robot Tracker')
    parser.add_argument('input_filename', type=str, help='input file name')
    parser.add_argument('-o', '-output_filename', type=str, default='prediction.txt', help='output file name')
    parser.add_argument('-v', '--visualize', type=bool, default=False, help='whether to visualize output')
    parser.add_argument('-t', '--test', type=bool, default=False, help='use the last 2 seconds of the file as a test and outputs predicted error')
    args = parser.parse_args()
    data = readData(args.input_filename)
    trainingData = data
    testData = None
    if args.test:
        # If in testing mode, we want to use the last 60 frames as test data
        trainingData = data[:-60]
        testData = data[len(data)-60:]
    predictions = predict(trainingData,args.visualize)
    assert len(predictions) == 60
    #for element in predictions:



if __name__ == '__main__':
    main()
