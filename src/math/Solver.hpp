//
// Created by gj on 4/29/24.
//

#ifndef VR_ARM_SOLVER_HPP
#define VR_ARM_SOLVER_HPP

namespace math {

    template<typename input ,typename solution,typename param>
    class Solver {

    public:
        Solver();
        ~Solver();

        virtual void addParam(param &&paramInput);
        virtual void clearParam();

        virtual void solve();
        virtual void apply(solution &cp);
        virtual void getInput(input&& in);

    private:
        param param_;
        solution solution_;
    };
}

#endif //VR_ARM_SOLVER_HPP
