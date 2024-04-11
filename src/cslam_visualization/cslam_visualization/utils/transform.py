#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Université de Lorraine - LORIA
# Fichier d'init du module Pose Transform :
#  -> matrices de transformations entre repères à partir d'une pose 3D
# utilisant le module numpy
#
###############################################################################+

# Modified by Pierre-Yves Lajoie

###############################################################################
# Chargement des modules standards
###############################################################################
import numpy as np
import quaternion
import math
from geometry_msgs.msg import Transform as TransformMsg
###############################################################################

###############################################################################
# DÉCLARATION DES TYPES DE DONNÉES
#
###############################################################################
# Description d'une transformation
###############################################################################
class Transform:
    # self.matrix = None  # matrix de transfo

    # Constructeur
    def __init__(self, mat=None, quat=None, pos=None):
        u""" Constructeur depuis une matrice OU un quaternion et une position."""
        assert(not(mat is not None and (quat is not None or pos is not None))), ''
        "Spécification exclusive matrix OU (quaternion + position) !"
        if mat is None:
            self.matrix = np.identity(4)
            if quat is not None and pos is not None:
                # (w, x, y, z)
                quat = np.asarray(quat)
                npquat = quaternion.quaternion(quat[0], quat[1],
                                               quat[2], quat[3])
                self.matrix[:3, :3] = quaternion.as_rotation_matrix(npquat)
                assert(len(pos) == 3)
                if len(pos) == 3:
                    self.matrix[0, 3] = pos[0]
                    self.matrix[1, 3] = pos[1]
                    self.matrix[2, 3] = pos[2]
        else:
            self.matrix = np.copy(mat)

    def __str__(self):
        u"""Affichage de la transformation."""
        return self.matrix.__str__()

    def __repr__(self):
        u"""Représentation interne de la classe."""
        return self.matrix.__repr__()

    def quat_2_mat(self, quat, pos):
        u"""Conversion quaternion vers matrix."""
        self.matrix[:3, :3] = quaternion.as_rotation_matrix(quat)
        self.matrix[:3, 3] = pos

    def inverse(self):
        u"""Inverse de la transformation."""
        return Transform(np.linalg.inv(self.matrix))

    def __invert__(self):
        u"""Inverse de la transformation inplace."""
        return Transform(np.linalg.inv(self.matrix))

    def __sub__(self, other):
        u"""Renvoie la transformation dans self du repère à l'origine de la transformation other."""
        return self.composition(~other)

    def __isub__(self, other):
        u"""Version 'inplace' de sub."""
        self = self.composition(~other)
        return self

    def quaternion(self):
        u"""Extraction du quaternion depuis matrix."""
        return quaternion.from_rotation_matrix(self.matrix)

    def position(self):
        u"""Extraction de la position depuis matrix."""
        return self.matrix[:3, 3]

    def composition(self, tr):
        u"""Composition de transformations."""
        return Transform(mat=np.dot(self.matrix, tr.matrix))

    def __mul__(self, other):
        u"""Composition de la transformation de other dans self."""
        return self.composition(other)

    def __imul__(self, other):
        u""""Version 'inplace' de mul."""
        self.matrix = self.matrix.dot(other.matrix)
        return self

    def relative_transform(self, other):
        u"""Transformation de self dans le repère other."""
        return ~other.composition(self)

    def projection(self, pt):
        u"""Transformation d'un point."""
        if (len(pt) == 3):
            return self.matrix.dot(pt + [1])
        else:
            return self.matrix.dot(pt)

    def to_msg(self):
        u"""Conversion en message ROS."""
        msg = TransformMsg()
        msg.translation.x = self.matrix[0, 3]
        msg.translation.y = self.matrix[1, 3]
        msg.translation.z = self.matrix[2, 3]
        msg.rotation.x = self.quaternion().x
        msg.rotation.y = self.quaternion().y
        msg.rotation.z = self.quaternion().z
        msg.rotation.w = self.quaternion().w
        return msg

def rotation_matrix(axe, angle):
    u"""Génére une matrice de rotation depuis un axe ('x','y' ou 'z') et un angle."""
    matrix = np.identity(4)
    if axe == 'x':
        matrix[1, 1] = math.cos(angle)
        matrix[1, 2] = -math.sin(angle)
        matrix[2, 1] = math.sin(angle)
        matrix[2, 2] = math.cos(angle)
    elif axe == 'y':
        matrix[0, 0] = math.cos(angle)
        matrix[0, 2] = math.sin(angle)
        matrix[2, 0] = -math.sin(angle)
        matrix[2, 2] = math.cos(angle)
    elif axe == 'z':
        matrix[0, 0] = math.cos(angle)
        matrix[0, 1] = -math.sin(angle)
        matrix[1, 0] = math.sin(angle)
        matrix[1, 1] = math.cos(angle)
    return matrix


def translation_matrix(tr):
    u"""Génére une matrice de translation depuis le vecteur tr."""
    matrix = np.identity(4)
    matrix[:3, 3] = np.asarray(tr)
    return matrix