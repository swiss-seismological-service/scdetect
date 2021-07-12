#ifndef SCDETECT_APPS_SCDETECT_BUILDER_IPP_
#define SCDETECT_APPS_SCDETECT_BUILDER_IPP_

template <typename TProduct>
std::unique_ptr<TProduct> Builder<TProduct>::build() {
  finalize();
  return std::move(_product);
}

template <typename TProduct>
void Builder<TProduct>::finalize() {}

#endif  // SCDETECT_APPS_SCDETECT_BUILDER_IPP_
