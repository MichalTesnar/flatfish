from uq_model import AIOModel
import uuid
import shutil


def main(args=None):
    my_model = AIOModel(training_set=([1, 2, 3], [4, 5, 6]))
    id_of_weights = str(uuid.uuid4())
    model_json = my_model.model.save_weights(id_of_weights)
    print("Model weights saved to {}".format(id_of_weights))
    my_model.model.load_weights(id_of_weights)
    print("Model weights loaded from {}".format(id_of_weights))
    shutil.rmtree(id_of_weights)
    print("Model weights deleted")


if __name__ == '__main__':
    main()
