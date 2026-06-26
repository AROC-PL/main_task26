class EMAFilter:
    def __init__(self, alpha=0.2):
        """
        alpha = 0.1 -> sangat halus
        alpha = 0.2 -> rekomendasi
        alpha = 0.3 -> lebih responsif
        """
        self.alpha = alpha
        self.value = None

    def update(self, measurement):
        if self.value is None:
            self.value = float(measurement)
        else:
            self.value = (
                self.alpha * measurement +
                (1.0 - self.alpha) * self.value
            )

        return self.value

    def reset(self):
        self.value = None