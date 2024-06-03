import numpy as np

UNCERTAINTY_COEFFICIENT = 1
PRIORITY_COEFFICIENT = 0

class ReplayBuffer():
    def __init__(self, model, capacity, mode='uniform'):
        # self.mode = mode
        self.mode = "score" # "uniform" or "score"
        # self.mode = "uniform"
        self.model = model
        self.capacity = capacity
        self.buffer = []
        self.position = 0

        self.alpha = 0.7
        self.epsilon = 1e-5
        
        self.beta_schedule = lambda episode: min(1.0, 0.4 + episode * (1.0 - 0.4) / 1000)  # linear schedule

    def push(self, sample):
        if len(self.buffer) < self.capacity:
            self.buffer.append(sample)
        else:
            self.buffer[self.position] = sample

        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, episode_number):
        """
        Sample a batch of episodes from the replay buffer.
        """
        if self.mode == 'uniform':
            indices = np.random.choice(len(self.buffer), batch_size, replace=False)
            samples = [self.buffer[index] for index in indices]

            weights = np.ones(batch_size)
            return samples, weights, 1
        elif self.mode == 'score':
            
            scores = np.zeros(len(self.buffer), dtype=np.float32)
            for i, (sample, target) in enumerate(self.buffer):
                pred_mean, pred_std = self.model.predict(sample.reshape(1, -1))
                uncertainty = float(np.mean(pred_std))
                priority = float(np.sum(np.square(target - pred_mean)))
                
                score = PRIORITY_COEFFICIENT * priority + UNCERTAINTY_COEFFICIENT * uncertainty

                scores[i] = np.abs(score) + self.epsilon

            beta = self.beta_schedule(episode_number)
            probabilities = scores ** self.alpha
            probabilities /= probabilities.sum()

            indices = np.random.choice(len(self.buffer), size=batch_size, p=probabilities)
            samples = [self.buffer[i] for i in indices]
            # weights = (len(self.buffer) * probabilities[indices]) ** (-beta)
            weights = probabilities[indices]
            weights /= weights.max()     # normalize weights
            maximum_score = np.max(scores[indices])
            return samples, weights, maximum_score