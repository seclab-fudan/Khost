#ifndef COVERAGE_H
#define COVERAGE_H

#include "en_global.h"
#include "en_config.h"
#include "addition.h"

namespace khost {

class Loader;

class CoverageAddition : public Addition {
public:
    CoverageAddition(Loader *loader) : Addition(loader) {}
    virtual ~CoverageAddition() {}
    virtual void *bytecode() override final;
    virtual void set_addr(uint32_t addr) override final;
private:
    virtual uint32_t calc_size() override final;
};

}  // namespace khost

#endif  // COVERAGE_H