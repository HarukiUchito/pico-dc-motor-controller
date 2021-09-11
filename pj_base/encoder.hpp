#ifndef ENCODER_HPP
#define ENCODER_HPP

class Encoder {
    public:
        Encoder(unsigned int inA, unsigned int inB);
        int get_count() const { return mCnt; };
        double get_rps() const { return mRPS; };
        void update();
        double get_dt() const { return mDt; };

        void pulse_callback();

        unsigned int mInA;
        double mCnt;
        double mLastCnt;
    private:
        unsigned int mInB;
        
        double mLastTime, mDt, mRPS;
};

#endif