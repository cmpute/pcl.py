from cython.operator cimport dereference as deref
from libcpp cimport bool
from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport VectorXf
from pcl.common cimport _ensure_true
from pcl.common.point_types cimport PointXYZ
from pcl.common.point_cloud cimport PointCloud as cPointCloud
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.conversions cimport fromPCLPointCloud2, toPCLPointCloud2
from pcl.PointCloud cimport PointCloud

from pcl.sample_consensus.model_types cimport SacModel
from pcl.sample_consensus.sac_model cimport SampleConsensusModel as cSampleConsensusModel
from pcl.sample_consensus.sac_model_plane cimport SampleConsensusModelPlane as cSampleConsensusModelPlane

from pcl.sample_consensus.sac cimport SampleConsensus as cSampleConsensus
from pcl.sample_consensus.ransac cimport RandomSampleConsensus as cRandomSampleConsensus

cdef inline cSampleConsensusModel[PointXYZ]* SampleConsensusModel_ptr(SampleConsensusModel obj):
    return <cSampleConsensusModel[PointXYZ]*>obj._ptr.get()

cdef class SampleConsensusModel:
    cdef shared_ptr[cSampleConsensusModel[PointXYZ]] _ptr
    cdef PointCloud _input

    def __init__(self, PointCloud cloud, indices=None, bool random=False):
        self._ptr = shared_ptr[cSampleConsensusModel[PointXYZ]](NULL)
        raise NotImplementedError("Abstract class cannot be initialized")

    property InputCloud:
        def __get__(self):
            return self._input
        def __set__(self, PointCloud value):
            cdef shared_ptr[cPointCloud[PointXYZ]] converted = make_shared[cPointCloud[PointXYZ]]()
            fromPCLPointCloud2(deref(value._ptr), deref(converted))
            SampleConsensusModel_ptr(self).setInputCloud(<shared_ptr[const cPointCloud[PointXYZ]]> converted)
            self._input = value

    property Indices:
        def __get__(self):
            return deref(SampleConsensusModel_ptr(self).getIndices().get())
        def __set__(self, indices):
            cdef shared_ptr[vector[int]] vec = make_shared[vector[int]]()
            for idx in indices:
                vec.get().push_back(idx)
            SampleConsensusModel_ptr(self).setIndices(vec)

    property ModelType:
        def __get__(self):
            return SampleConsensusModel_ptr(self).getModelType()
    property SampleSize:
        def __get__(self):
            return SampleConsensusModel_ptr(self).getSampleSize()

    property RadiusLimits:
        def __get__(self):
            cdef double lmin=0, lmax=0
            SampleConsensusModel_ptr(self).getRadiusLimits(lmin, lmax)
            return (lmin, lmax)
        def __set__(self, value):
            SampleConsensusModel_ptr(self).setRadiusLimits(value[0], value[1])
    
    def getSamplesMaxDist(self, double radius):
        return SampleConsensusModel_ptr(self).getSamplesMaxDist(radius)

cdef class SampleConsensusModelPlane(SampleConsensusModel):
    def __init__(self, PointCloud cloud, indices=None, bool random=False):
        cdef shared_ptr[cPointCloud[PointXYZ]] converted = make_shared[cPointCloud[PointXYZ]]()
        fromPCLPointCloud2(deref(cloud._ptr), deref(converted))
        self._input = cloud
        
        if indices != None:
            self._ptr = shared_ptr[cSampleConsensusModel[PointXYZ]](new cSampleConsensusModelPlane[PointXYZ](converted, indices, random))
        else: self._ptr = shared_ptr[cSampleConsensusModel[PointXYZ]](new cSampleConsensusModelPlane[PointXYZ](converted, random))


cdef inline cSampleConsensus[PointXYZ]* SampleConsensus_ptr(SampleConsensus obj):
    return <cSampleConsensus[PointXYZ]*>obj._ptr.get()

cdef class SampleConsensus:
    cdef shared_ptr[cSampleConsensus[PointXYZ]] _ptr
    cdef SampleConsensusModel _sac_model

    def __init__(self, SampleConsensusModel model, threshold=None, bool random=False):
        self._ptr = shared_ptr[cSampleConsensus[PointXYZ]](NULL)

    property SampleConsensusModel:
        def __get__(self):
            return self._sac_model
        def __set__(self, SampleConsensusModel model):
            self._sac_model = model
            SampleConsensus_ptr(self).setSampleConsensusModel(model._ptr)

    property DistanceThreshold:
        def __get__(self):
            return SampleConsensus_ptr(self).getDistanceThreshold()
        def __set__(self, double value):
            SampleConsensus_ptr(self).setDistanceThreshold(value)

    property MaxIterations:
        def __get__(self):
            return SampleConsensus_ptr(self).getMaxIterations()
        def __set__(self, int value):
            SampleConsensus_ptr(self).setMaxIterations(value)

    property Probability:
        def __get__(self):
            return SampleConsensus_ptr(self).getProbability()
        def __set__(self, double value):
            SampleConsensus_ptr(self).setProbability(value)

    property Model:
        def __get__(self):
            cdef vector[int] retval
            SampleConsensus_ptr(self).getModel(retval)
            return retval
    property Inliers:
        def __get__(self):
            cdef vector[int] retval
            SampleConsensus_ptr(self).getInliers(retval)
            return retval
    property ModelCoefficients:
        def __get__(self):
            cdef VectorXf retval
            cdef vector[float] retvec
            SampleConsensus_ptr(self).getModelCoefficients(retval)
            retvec.reserve(retval.size())
            for i in range(retval.size()):
                retvec.push_back(retval.element(i))
            return retvec

    def computeModel(self, int debug_verbosity_level=0):
        _ensure_true(SampleConsensus_ptr(self).computeModel(debug_verbosity_level), "computeModel")

    def refineModel(self, double sigma=3.0, int max_iterations=10000):
        _ensure_true(SampleConsensus_ptr(self).refineModel(sigma, max_iterations), "refineModel")

cdef class RandomSampleConsensus(SampleConsensus):
    def __init__(self, SampleConsensusModel model, threshold=None):
        self._sac_model = model
        
        if threshold != None:
            self._ptr = shared_ptr[cSampleConsensus[PointXYZ]](new cRandomSampleConsensus[PointXYZ](model._ptr, threshold))
        else: self._ptr = shared_ptr[cSampleConsensus[PointXYZ]](new cRandomSampleConsensus[PointXYZ](model._ptr))
