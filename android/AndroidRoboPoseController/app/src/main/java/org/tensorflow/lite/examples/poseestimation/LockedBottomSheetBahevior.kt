package org.tensorflow.lite.examples.poseestimation

import android.content.Context
import android.util.AttributeSet
import android.view.View
import com.google.android.material.bottomsheet.BottomSheetBehavior

class LockedBottomSheetBahevior<V : View> : BottomSheetBehavior<V> {
    constructor() : super() {

        isDraggable = false
    }


    constructor(context: Context, attrs: AttributeSet) : super(context, attrs) {
        isDraggable = false
    }
}