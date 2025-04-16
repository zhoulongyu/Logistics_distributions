import flwr as fl
from typing import List, Tuple
import numpy as np

def get_eval_fn(model):
    def evaluate(weights: List[np.ndarray]) -> Tuple[float, float]:
        model.set_weights(weights)
        loss, accuracy = model.evaluate(x_test, y_test)
        return loss, accuracy
    return evaluate

if __name__ == "__main__":
    # Load your global model here
    model = ...  # Replace with your model initialization
    x_test, y_test = ...  # Replace with your test data

    strategy = fl.server.strategy.FedAvg(
        eval_fn=get_eval_fn(model),
        fraction_fit=0.1,
        fraction_eval=0.1,
        min_fit_clients=2,
        min_eval_clients=2,
        min_available_clients=2,
    )

    fl.server.start_server(
        server_address="0.0.0.0:8080",
        config=fl.server.ServerConfig(num_rounds=3),
        strategy=strategy,
    )
