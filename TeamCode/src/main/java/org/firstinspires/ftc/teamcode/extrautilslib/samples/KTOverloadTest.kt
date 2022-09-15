@file:Suppress("FINITE_BOUNDS_VIOLATION_IN_JAVA")

package com.icaras84.extrautilslib.samples

import com.icaras84.extrautilslib.core.maths.vectors.EULVector
import com.icaras84.extrautilslib.core.maths.vectors.times

class KTOverloadTest<T : EULVector<*>>(input1 : T, input2 : T) {

    private var vector1 : T
    private var vector2 : T

    init {
        this.vector1 = input1
        this.vector2 = input2
    }

    fun process() : EULVector<*> {
        return vector1 + vector2
    }
}