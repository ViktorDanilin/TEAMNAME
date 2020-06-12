from imageai.Prediction.Custom import CustomImagePrediction
import os

execution_path = os.getcwd()

prediction = CustomImagePrediction()
prediction.setModelTypeAsResNet()
prediction.setModelPath(os.path.join(execution_path, "znaki_fit/models/model_ex-002_acc-0.214286.h5"))
prediction.setJsonPath(os.path.join(execution_path, "znaki_fit/json/model_class.json"))
prediction.loadModel(num_objects=4)

predictions, probabilities = prediction.predictImage(os.path.join(execution_path, "obj.jpg"), result_count=1)

for eachPrediction, eachProbability in zip(predictions, probabilities):
    print(eachPrediction , " : " , eachProbability)