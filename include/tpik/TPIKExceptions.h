#ifndef __TPIKEXCEPTIONS_H__
#define __TPIKEXCEPTIONS_H__

#include <iostream>

namespace tpik {

class ExceptionWithHow : public std::exception {
public:
    void SetHow(std::string how)
    {
        how_ = how;
    }
    const char* how() const noexcept
    {
        return how_.c_str();
    }

private:
    std::string how_;
};

class NotInitialziedTaskParameterException : public ExceptionWithHow {

    virtual const char* what() const noexcept
    {
        return "Cannot update the task, either not initialzed or wrong task parameters (check how())";
    }
};

class ActionManagerException : public ExceptionWithHow {

    virtual const char* what() const noexcept
    {
        return "[ActionManger] Hierarchy/Action exception (check how())";
    }
};
}

#endif
