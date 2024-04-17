import numpy as np


class ReplayBuffer():
    def __init__(self, capacity, mode='uniform'):
        self.mode = mode
        self.capacity = capacity
        self.buffer = []
        self.position = 0

        self.alpha = 0.6
        self.epsilon = 1e-5
        self.scores = np.zeros(capacity, dtype=np.float32)
        self.beta_schedule = lambda episode: min(1.0, 0.4 + episode * (1.0 - 0.4) / 1000)  # linear schedule

    def push(self, sample, score):
        if len(self.buffer) < self.capacity:
            self.buffer.append(sample)
        else:
            self.buffer[self.position] = sample

        self.scores[self.position] = np.abs(score) + self.epsilon
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, episode_number):
        """
        Sample a batch of episodes from the replay buffer.
        """
        if self.mode == 'uniform':
            indices = np.random.choice(len(self.buffer), batch_size, replace=False)
            samples = [self.buffer[index] for index in indices]
            for sample in samples:
                sample.training_weight = 1
            return samples
        elif self.mode == 'score':
            beta = self.beta_schedule(episode_number)
            probabilities = self.scores[:len(self.buffer)] ** self.alpha
            probabilities /= probabilities.sum()

            indices = np.random.choice(len(self.buffer), size=batch_size, p=probabilities)
            samples = [self.buffer[i] for i in indices]
            weights = (len(self.buffer) * probabilities[indices]) ** (-beta)
            weights /= weights.max()     # normalize weights
            for sample, weight in zip(samples, weights):
                sample.training_weight = float(weight)
            return samples