from . import features
from . import io



def apply_parsers(functions, config):
    """Return a new dictionary with the same keys as config, but with the
    values replaced by applying the corresponding function in
    functions where defined.
    """
    def id_(x): return x
    return {k: functions.get(k, id_)(v) for k, v in config.items()}

def feature_input(config):
    """Given a configuration dictionary, create a features.FeatureInput"""
    return io.feature_input_from_files(**config)

def feature_functions(config):
    """Given a list of configuration dictionaries, return a list of
    feature functions

    """
    return [feature_function(c) for c in config]

def feature_function(config):
    """Given a configuration dictionary, create a feature function"""
    config = config.copy()
    f = getattr(features, config.pop('name'))

    # Set of functions that have a "feature" argument that should be
    # passed through config_feature
    if f in {features.log}:
        config['feature'] = feature_function(config['feature'])
    return f(**config)

def road_segment_filter(config):
    """Given a configuration dictionary, return arguments to
    filter_road_segments
    """
    if config is None:
        return None
    return apply_parsers(dict(feature=feature_function), config)