from imageai.Prediction.Custom import ModelTraining

model_trainer = ModelTraining()
model_trainer.setModelTypeAsResNet()
model_trainer.setDataDirectory(r"/home/viktor/RRO_2020/TEAMNAME/znaki/znaki_fit")
model_trainer.trainModel(num_objects=5, num_experiments=150, enhance_data=True, batch_size=8, show_network_summary=True)


