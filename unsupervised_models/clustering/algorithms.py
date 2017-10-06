from sklearn.cluster import KMeans
import numpy as np
import abc

def transform_data(decorated):

    def decorator(*args, **kwargs):
        args = list(args)
        args[1] = np.asarray(args[1])
        return decorated(*args, **kwargs)

    return decorator

class BBoxClusteringAlg(object):


    def __init__(self):
        super(BBoxClusteringAlg, self).__init__()

    @abc.abstractmethod
    @transform_data
    def fit(self, X, n_clusters):
        """
        Fits the data to n clusters
        :param X: List of data points
        :param n_clusters:
        :return: None
        """
        pass

    @abc.abstractmethod
    @transform_data
    def fit_predict(self, X):
        """
        Predicts and fits the cluster further
        :param X: List of data points
        :return: Array of cluster ID's
        """
        pass

    @abc.abstractmethod
    @transform_data
    def predict(self, X):
        """
        Returns a list of cluster ID's based on the data
        :param X: List of data points
        :return: Array of cluster ID's
        """
        pass


class BBoxKMeansClustering(BBoxClusteringAlg):


    def __init__(self):
        super(BBoxKMeansClustering, self).__init__()

    @transform_data
    def fit(self, X, n_clusters, max_iter=300):
        self.alg = KMeans(n_clusters=n_clusters, max_iter=max_iter, precompute_distances=True, n_jobs=4)
        self.alg.fit(X)

    @transform_data
    def fit_predict(self, X):
        return self.alg.fit_predict(X)

    @transform_data
    def predict(self, X):
        return self.alg.predict(X)




if __name__ == "__main__":


    # Creating the algorithm
    alg = BBoxKMeansClustering()
    data = np.asarray(np.random.normal(0,1,(100,100)))
    # Clustering based on data
    print(alg.fit(data, n_clusters=2))

    # Predicting the clusters
    print(alg.predict([data[30]]))




